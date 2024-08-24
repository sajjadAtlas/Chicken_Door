/* LEDC (LED Controller) basic example
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "driver/ledc.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include <inttypes.h>
#include <unistd.h>
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "sdkconfig.h"
#include "../../../components/freertos/FreeRTOS-Kernel-SMP/include/freertos/queue.h"
#include <nvs_flash.h>
#include <esp_rmaker_core.h>
#include <esp_rmaker_standard_types.h>
#include <esp_rmaker_standard_params.h>
#include <esp_rmaker_standard_devices.h>
#include <esp_rmaker_schedule.h>
#include <esp_rmaker_scenes.h>
#include <esp_rmaker_console.h>
#include <esp_rmaker_ota.h>
#include <esp_debug_helpers.h>
#include <esp_rmaker_common_events.h>

#include <app_wifi.h>
#include <app_insights.h>
#include <string.h>

#include <iot_button.h>
#include <app_reset.h>
#include <ws2812_led.h>
#include <stdbool.h>
#include "esp_pm.h"
//#include "app_priv.h"
#include "esp_wifi.h"


#define DEFAULT_POWER  true
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO_0        (5) // Define the output GPIO
#define LEDC_OUTPUT_IO_1        (6) 
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (8192) // Set duty to 50%. (2 ** 13) * 50% = 4096
#define LEDC_FREQUENCY          (4000) // Frequency in Hertz. Set frequency at 1 kHz
#define GPIO_OUTPUT_IO_1        (3)
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_IO_1))
#define ENCODER_PIN         (2)
#define SWITCH_PIN          (10)
#define GPIO_INPUT_PIN_SEL  ((1ULL<<ENCODER_PIN)||(1ULL<<SWITCH_PIN))


#define UP 1
#define DOWN 0
#define UP_CHANNEL LEDC_CHANNEL_0
#define DOWN_CHANNEL LEDC_CHANNEL_1
#define ESP_INTR_FLAG_DEFAULT 0
#define ON 1
#define OFF 0


#define BUTTON_GPIO          CONFIG_EXAMPLE_BOARD_BUTTON_GPIO
#define BUTTON_ACTIVE_LEVEL  0

/* This is the GPIO on which the power will be set */
#define OUTPUT_GPIO    CONFIG_EXAMPLE_OUTPUT_GPIO
static bool g_power_state = DEFAULT_POWER;

/* These values correspoind to H,S,V = 120,100,10 */
#define DEFAULT_RED     0
#define DEFAULT_GREEN   25
#define DEFAULT_BLUE    0
#define WIFI_RESET_BUTTON_TIMEOUT       3
#define FACTORY_RESET_BUTTON_TIMEOUT    10
static QueueHandle_t gpio_evt_queue = NULL;
static QueueHandle_t encoder_queue = NULL;
 volatile int cnt = 0;
COREDUMP_DRAM_ATTR volatile int flag = 1;
volatile int direction = DOWN;
volatile int motorStopped = 1;
 void motor_stop();
void motor_start(int direction);
void app_driver_init(void);
int app_driver_set_state(bool state);
bool app_driver_get_state(void);
static const char *TAG = "app_main";
esp_rmaker_device_t *switch_device;
static esp_pm_lock_handle_t s_pm_apb_lock   = NULL;

static void app_indicator_set(bool state)
{
    if(motorStopped==1){
        if (state) {
            ws2812_led_set_rgb(DEFAULT_RED, DEFAULT_GREEN, DEFAULT_BLUE);
            gpio_set_intr_type(ENCODER_PIN, GPIO_INTR_NEGEDGE);
            motor_start(direction);
            printf("initial direction: %d\n", direction);
            direction = !direction;
            printf("direction: %d\n", direction);
            motorStopped = 0;
        } else {
            ws2812_led_set_rgb(25, 0, 0);
            gpio_set_intr_type(ENCODER_PIN, GPIO_INTR_NEGEDGE);

            motor_start(direction);
            printf("initial direction: %d\n", direction);
            direction = !direction;
            printf("direction: %d\n", direction);
            motorStopped = 0;

        }
            
    }
} 


static void app_indicator_init(void)
{
    ws2812_led_init();
    //app_indicator_set(g_power_state);
}
static void push_btn_cb(void *arg)
{
    bool new_state = !g_power_state;
    app_driver_set_state(new_state);
#ifdef CONFIG_EXAMPLE_ENABLE_TEST_NOTIFICATIONS
    /* This snippet has been added just to demonstrate how the APIs esp_rmaker_param_update_and_notify()
     * and esp_rmaker_raise_alert() can be used to trigger push notifications on the phone apps.
     * Normally, there should not be a need to use these APIs for such simple operations. Please check
     * API documentation for details.
     */
    if (new_state) {
        esp_rmaker_param_update_and_notify(
                esp_rmaker_device_get_param_by_name(switch_device, ESP_RMAKER_DEF_POWER_NAME),
                esp_rmaker_bool(new_state));
    } else {
        esp_rmaker_param_update_and_report(
                esp_rmaker_device_get_param_by_name(switch_device, ESP_RMAKER_DEF_POWER_NAME),
                esp_rmaker_bool(new_state));
        esp_rmaker_raise_alert("Switch was turned off");
    }
#else
    esp_rmaker_param_update_and_report(
            esp_rmaker_device_get_param_by_name(switch_device, ESP_RMAKER_DEF_POWER_NAME),
            esp_rmaker_bool(new_state));
#endif
}

static void set_power_state(bool target)
{
    gpio_set_level(OUTPUT_GPIO, target);
    app_indicator_set(target);
}

void app_driver_init()
{
    button_handle_t btn_handle = iot_button_create(BUTTON_GPIO, BUTTON_ACTIVE_LEVEL);
    if (btn_handle) {
        /* Register a callback for a button tap (short press) event */
        iot_button_set_evt_cb(btn_handle, BUTTON_CB_TAP, push_btn_cb, NULL);
        /* Register Wi-Fi reset and factory reset functionality on same button */
        app_reset_button_register(btn_handle, WIFI_RESET_BUTTON_TIMEOUT, FACTORY_RESET_BUTTON_TIMEOUT);
    }

    /* Configure power */
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 1,
    };
    io_conf.pin_bit_mask = ((uint64_t)1 << OUTPUT_GPIO);
    /* Configure the GPIO */
    gpio_config(&io_conf);
    app_indicator_init();
}

int IRAM_ATTR app_driver_set_state(bool state)
{
    if(g_power_state != state) {
        g_power_state = state;
        set_power_state(g_power_state);
    }
    return ESP_OK;
}

bool app_driver_get_state(void)
{
    return g_power_state;
}




/* Callback to handle commands received from the RainMaker cloud */
static esp_err_t write_cb(const esp_rmaker_device_t *device, const esp_rmaker_param_t *param,
            const esp_rmaker_param_val_t val, void *priv_data, esp_rmaker_write_ctx_t *ctx)
{
    if (ctx) {
        ESP_LOGI(TAG, "Received write request via : %s", esp_rmaker_device_cb_src_to_str(ctx->src));
    }
    if (strcmp(esp_rmaker_param_get_name(param), ESP_RMAKER_DEF_POWER_NAME) == 0) {
        ESP_LOGI(TAG, "Received value = %s for %s - %s",
                val.val.b? "true" : "false", esp_rmaker_device_get_name(device),
                esp_rmaker_param_get_name(param));
        app_driver_set_state(val.val.b);
        esp_rmaker_param_update(param, val);
    }
    return ESP_OK;
}
static void IRAM_ATTR pinch_timer(TimerHandle_t xTimer){
    printf("pinched\n");
    motor_stop();
    motorStopped = 1;

    xTimerStop(xTimer, 0);
    vTaskDelay(2000/portTICK_PERIOD_MS);
    //printf("flag: %d\n", flag);
    cnt = 0;
    gpio_set_intr_type(SWITCH_PIN,  GPIO_INTR_POSEDGE);
    
}

TimerHandle_t timer = NULL;
static void IRAM_ATTR button_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;


    xQueueSendFromISR(gpio_evt_queue, &flag, NULL);
}
static void IRAM_ATTR encoder_isr_handler(void* arg){
    
    cnt++;
    gpio_set_intr_type(SWITCH_PIN,  GPIO_INTR_DISABLE);

    xQueueSendFromISR(encoder_queue, &cnt, NULL);

}


static void encoder_task(void* arg){

    uint32_t io_num;

    for(;;){
        if(xQueueReceive(encoder_queue, &io_num, portMAX_DELAY)){

           vTaskDelay(75/portTICK_PERIOD_MS);
           xTimerStart(timer, 0);
            printf("pulse count: %ld\n", io_num);
            if(io_num >= 40){
                cnt = 0;
                io_num = 0;
                gpio_set_intr_type(ENCODER_PIN, GPIO_INTR_DISABLE);
                
                
                //prevTime = 0;
                xTimerStop(timer, 0);
                motor_stop();

                motorStopped = 1;

                gpio_set_intr_type(SWITCH_PIN,  GPIO_INTR_POSEDGE);

            }
            


        }
        //xTimerStop(timer, 0);
        //prevTime = xTaskGetTickCount();

    }

}
static void gpio_task(void* arg)
{
    uint32_t io_num;

    for (;;) {


        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)){
            gpio_set_intr_type(SWITCH_PIN,  GPIO_INTR_DISABLE);
                if(io_num && gpio_get_level(10) == 1){
                //printf("reached: %d\n", gpio_get_level(10));
                motor_start(direction); 
                motorStopped = 0;
                printf("initial direction: %d\n", direction);
                direction = !direction;
                printf("direction: %d\n", direction);

                flag = 0;
                gpio_set_intr_type(ENCODER_PIN, GPIO_INTR_NEGEDGE);
            }
            else if(io_num == 0 && gpio_get_level(10) == 1){
                //printf("reached 2nd: %d\n", gpio_get_level(10));
                motor_start(direction);
                motorStopped = 0;
                printf("initial direction: %d\n", direction);
                direction = !direction;
                printf("direction: %d\n", direction);
                flag = 1;
                gpio_set_intr_type(ENCODER_PIN, GPIO_INTR_NEGEDGE);

            }

        }

    }
}


static void ledc_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer0 = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg = 0
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer0));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel0 = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_0,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO_0,
        .duty           = 0, // Set duty to 50%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel0));


//////////////////////////////////////////////////////////////////////


    ledc_timer_config_t ledc_timer1 = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg = 0

    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer1));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel1 = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_1,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO_1,
        .duty           = 0, // Set duty to 50%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel1));
}

 void motor_start(int direction){
    ESP_ERROR_CHECK(esp_pm_lock_acquire(s_pm_apb_lock));
        ESP_ERROR_CHECK(esp_pm_dump_locks(stdout));

    if(direction == UP){

        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, 8192));

        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0));

        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_1, 0));

        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_1));
    }

    else if(direction == DOWN){

        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_1, 8192));

        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_1));

        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, 0));

        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0));

        
    }

}

 void motor_stop(){

    ESP_ERROR_CHECK(esp_pm_lock_release(s_pm_apb_lock));
    ESP_ERROR_CHECK(esp_pm_dump_locks(stdout));

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, UP_CHANNEL, 8192));

    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, UP_CHANNEL));

////////////////////////////////////////////////////////////////////

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, DOWN_CHANNEL, 8192));

    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, DOWN_CHANNEL));

}

static void gpio_init(){

    gpio_config_t io_conf = {};
    //disable interrupt


    //interrupt of rising edge
    io_conf.intr_type = 0;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    gpio_set_level(GPIO_OUTPUT_IO_1, 1);


    //interrupt of rising edge
    io_conf.intr_type = 0;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 0;
    io_conf.pull_down_en=0; 
    gpio_config(&io_conf);

    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(ENCODER_PIN, encoder_isr_handler, (void*) ENCODER_PIN);
    gpio_isr_handler_add(SWITCH_PIN, button_isr_handler, (void*) SWITCH_PIN);


}



void app_main(void)
{


    gpio_init();
    esp_pm_lock_create(ESP_PM_APB_FREQ_MAX, 0, "l_apb", &s_pm_apb_lock);
    //gpio_set_intr_type(ENCODER_PIN, GPIO_INTR_DISABLE);
    gpio_evt_queue = xQueueCreate(1, sizeof(uint32_t));
    encoder_queue = xQueueCreate(1, sizeof(uint32_t));
    xTaskCreate(gpio_task, "gpio_task", 2048, NULL, 15, NULL); 
    xTaskCreate(encoder_task, "encoder_task", 2048, NULL, 6, NULL);  
    ledc_init();
    timer =  xTimerCreate("pinch_timer",
                            pdMS_TO_TICKS(600),
                            pdTRUE,
                            1,
                            pinch_timer);
    //calibrate();

     //n
     //esp_rmaker_console_init();
    
    app_driver_init();
    
    app_driver_set_state(DEFAULT_POWER);

    /* Initialize NVS. */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );

    /* Initialize Wi-Fi. Note that, this should be called before esp_rmaker_node_init()
     */
    app_wifi_init();

    /* Register an event handler to catch RainMaker events */
    

    /* Initialize the ESP RainMaker Agent.
     * Note that this should be called after app_wifi_init() but before app_wifi_start()
     * */
    esp_rmaker_config_t rainmaker_cfg = {
        .enable_time_sync = false,
    };
    esp_rmaker_node_t *node = esp_rmaker_node_init(&rainmaker_cfg, "ESP RainMaker Device", "Switch");
    if (!node) {
        ESP_LOGE(TAG, "Could not initialise node. Aborting!!!");
        vTaskDelay(5000/portTICK_PERIOD_MS);
        abort();
    }

    /* Create a Switch device.
     * You can optionally use the helper API esp_rmaker_switch_device_create() to
     * avoid writing code for adding the name and power parameters.
     */
    switch_device = esp_rmaker_device_create("Switch", ESP_RMAKER_DEVICE_SWITCH, NULL);

    /* Add the write callback for the device. We aren't registering any read callback yet as
     * it is for future use.
     */
    esp_rmaker_device_add_cb(switch_device, write_cb, NULL);

    /* Add the standard name parameter (type: esp.param.name), which allows setting a persistent,
     * user friendly custom name from the phone apps. All devices are recommended to have this
     * parameter.
     */
    esp_rmaker_device_add_param(switch_device, esp_rmaker_name_param_create(ESP_RMAKER_DEF_NAME_PARAM, "Switch"));

    /* Add the standard power parameter (type: esp.param.power), which adds a boolean param
     * with a toggle switch ui-type.
     */
    esp_rmaker_param_t *power_param = esp_rmaker_power_param_create(ESP_RMAKER_DEF_POWER_NAME, DEFAULT_POWER);
    esp_rmaker_device_add_param(switch_device, power_param);


    /* Assign the power parameter as the primary, so that it can be controlled from the
     * home screen of the phone apps.
     */
    esp_rmaker_device_assign_primary_param(switch_device, power_param);

    /* Add this switch device to the node */
    esp_rmaker_node_add_device(node, switch_device);

    /* Enable OTA */
    esp_rmaker_ota_enable_default();

    /* Enable timezone service which will be require for setting appropriate timezone
     * from the phone apps for scheduling to work correctly.
     * For more information on the various ways of setting timezone, please check
     * https://rainmaker.espressif.com/docs/time-service.html.
     */
    esp_rmaker_timezone_service_enable();

    /* Enable scheduling. */
    esp_rmaker_schedule_enable();

    /* Enable Scenes */
    esp_rmaker_scenes_enable();

    /* Enable Insights. Requires CONFIG_ESP_INSIGHTS_ENABLED=y */
    app_insights_enable();

    /* Start the ESP RainMaker Agent */
    esp_rmaker_start();

    err = app_wifi_set_custom_mfg_data(MGF_DATA_DEVICE_TYPE_SWITCH, MFG_DATA_DEVICE_SUBTYPE_SWITCH);
    /* Start the Wi-Fi.
     * If the node is provisioned, it will start connection attempts,
     * else, it will start Wi-Fi provisioning. The function will return
     * after a connection has been successfully established
     */
    err = app_wifi_start(POP_TYPE_RANDOM);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not start Wifi. Aborting!!!");
        vTaskDelay(5000/portTICK_PERIOD_MS);
        abort();
    }
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_MAX_MODEM));
    
    // Configure dynamic frequency scaling:
    // maximum and minimum frequencies are set in sdkconfig,
    // automatic light sleep is enabled if tickless idle support is enabled.
    esp_pm_config_t pm_config = {
            .max_freq_mhz = 80, //Maximum CPU frequency
            .min_freq_mhz = 40,
            .light_sleep_enable = true

    };
    
    ESP_ERROR_CHECK( esp_pm_configure(&pm_config) );
    
    gpio_set_intr_type(SWITCH_PIN,  GPIO_INTR_POSEDGE);
    vTaskDelay(1);
}