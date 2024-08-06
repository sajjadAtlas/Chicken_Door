/* LEDC (LED Controller) basic example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
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

#include <esp_rmaker_common_events.h>

#include <app_wifi.h>
#include <app_insights.h>

#include "app_priv.h"


#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO_0        (5) // Define the output GPIO
#define LEDC_OUTPUT_IO_1        (6) 
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (4096) // Set duty to 50%. (2 ** 13) * 50% = 4096
#define LEDC_FREQUENCY          (1000) // Frequency in Hertz. Set frequency at 1 kHz
#define GPIO_OUTPUT_IO_1        (3)
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_IO_1))
#define ENCODER_PIN         (4)
#define SWITCH_PIN          (7)
#define GPIO_INPUT_PIN_SEL  ((1ULL<<ENCODER_PIN)||(1ULL<<SWITCH_PIN))


#define UP 1
#define DOWN 0
#define UP_CHANNEL LEDC_CHANNEL_0
#define DOWN_CHANNEL LEDC_CHANNEL_1
#define ESP_INTR_FLAG_DEFAULT 0
#define ON 1
#define OFF 0

/* Warning:
 * For ESP32, ESP32S2, ESP32S3, ESP32C3, ESP32C2, ESP32C6, ESP32H2, ESP32P4 targets,
 * when LEDC_DUTY_RES selects the maximum duty resolution (i.e. value equal to SOC_LEDC_TIMER_BIT_WIDTH),
 * 100% duty cycle is not reachable (duty cannot be set to (2 ** SOC_LEDC_TIMER_BIT_WIDTH)).
 */


static QueueHandle_t gpio_evt_queue = NULL;
static QueueHandle_t encoder_queue = NULL;
volatile int cnt = 0;
volatile int flag = 1;
volatile int prevTime = 0;
void motor_stop();
void motor_start(int direction);
static const char *TAG = "app_main";

esp_rmaker_device_t *switch_device;

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
static void pinch_timer(TimerHandle_t xTimer){
    printf("pinched\n");
    motor_stop();
    xTimerStop(xTimer, 0);
    vTaskDelay(3000/portTICK_PERIOD_MS);
    printf("flag: %d\n", flag);
    if(flag == 1){
        motor_start(!flag);
    }
    else{
        gpio_set_intr_type(SWITCH_PIN,  GPIO_INTR_POSEDGE);
        cnt = 0;
    }
}
TimerHandle_t timer = NULL;
volatile int appDriverFlag = 0;
static void IRAM_ATTR button_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    
     
    xQueueSendFromISR(gpio_evt_queue, &flag, NULL);
}
static void IRAM_ATTR encoder_isr_handler(void* arg){
    cnt++;
    
    //gpio_set_intr_type(SWITCH_PIN,  GPIO_INTR_DISABLE);

    xQueueSendFromISR(encoder_queue, &cnt, NULL);

}


static void encoder_task(void* arg){

    uint32_t io_num;

    for(;;){
        if(xQueueReceive(encoder_queue, &io_num, portMAX_DELAY)){
            
            //cnt++;
            printf("pulse count: %ld\n", io_num);
            xTimerStart(timer, 0);
            if(io_num == 40){
                
                cnt = 0;
                gpio_set_intr_type(SWITCH_PIN,  GPIO_INTR_POSEDGE);
                //prevTime = 0;
                xTimerStop(timer, 0);
                motor_stop();
                //gpio_set_intr_type(ENCODER_PIN, GPIO_INTR_DISABLE);
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
            appDriverFlag = 1;
            if(io_num){
                gpio_set_intr_type(SWITCH_PIN,  GPIO_INTR_DISABLE);
                motor_start(UP);
                flag = 0;
                gpio_set_intr_type(ENCODER_PIN, GPIO_INTR_NEGEDGE);
            }
            else if(io_num == 0){
                gpio_set_intr_type(SWITCH_PIN,  GPIO_INTR_DISABLE);
                motor_start(DOWN);
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
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 1 kHz
        
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
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 1 kHz
        
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

    if(direction == UP){

        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, UP_CHANNEL, 4096));
   
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, UP_CHANNEL));
    }
   
    else if(direction == DOWN){

        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, DOWN_CHANNEL, LEDC_DUTY));
   
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, DOWN_CHANNEL));
    }
   
}

void motor_stop(){

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, UP_CHANNEL, 0));

    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, UP_CHANNEL));

////////////////////////////////////////////////////////////////////

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, DOWN_CHANNEL, 0));

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
    io_conf.pull_up_en = 1;
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
    gpio_config(&io_conf);

    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(ENCODER_PIN, encoder_isr_handler, (void*) ENCODER_PIN);
    gpio_isr_handler_add(SWITCH_PIN, button_isr_handler, (void*) SWITCH_PIN);


}

static void calibrate(){
    gpio_set_intr_type(SWITCH_PIN,  GPIO_INTR_DISABLE);
    gpio_set_intr_type(ENCODER_PIN, GPIO_INTR_NEGEDGE);
    motor_start(UP);
    flag = 0;

}

/* Event handler for catching RainMaker events */
static void event_handler(void* arg, esp_event_base_t event_base,
                          int32_t event_id, void* event_data)
{
    if (event_base == RMAKER_EVENT) {
        switch (event_id) {
            case RMAKER_EVENT_INIT_DONE:
                ESP_LOGI(TAG, "RainMaker Initialised.");
                break;
            case RMAKER_EVENT_CLAIM_STARTED:
                ESP_LOGI(TAG, "RainMaker Claim Started.");
                break;
            case RMAKER_EVENT_CLAIM_SUCCESSFUL:
                ESP_LOGI(TAG, "RainMaker Claim Successful.");
                break;
            case RMAKER_EVENT_CLAIM_FAILED:
                ESP_LOGI(TAG, "RainMaker Claim Failed.");
                break;
            case RMAKER_EVENT_LOCAL_CTRL_STARTED:
                ESP_LOGI(TAG, "Local Control Started.");
                break;
            case RMAKER_EVENT_LOCAL_CTRL_STOPPED:
                ESP_LOGI(TAG, "Local Control Stopped.");
                break;
            default:
                ESP_LOGW(TAG, "Unhandled RainMaker Event: %"PRIi32, event_id);
        }
    } else if (event_base == RMAKER_COMMON_EVENT) {
        switch (event_id) {
            case RMAKER_EVENT_REBOOT:
                ESP_LOGI(TAG, "Rebooting in %d seconds.", *((uint8_t *)event_data));
                break;
            case RMAKER_EVENT_WIFI_RESET:
                ESP_LOGI(TAG, "Wi-Fi credentials reset.");
                break;
            case RMAKER_EVENT_FACTORY_RESET:
                ESP_LOGI(TAG, "Node reset to factory defaults.");
                break;
            case RMAKER_MQTT_EVENT_CONNECTED:
                ESP_LOGI(TAG, "MQTT Connected.");
                break;
            case RMAKER_MQTT_EVENT_DISCONNECTED:
                ESP_LOGI(TAG, "MQTT Disconnected.");
                break;
            case RMAKER_MQTT_EVENT_PUBLISHED:
                ESP_LOGI(TAG, "MQTT Published. Msg id: %d.", *((int *)event_data));
                break;
            default:
                ESP_LOGW(TAG, "Unhandled RainMaker Common Event: %"PRIi32, event_id);
        }
    } else if (event_base == APP_WIFI_EVENT) {
        switch (event_id) {
            case APP_WIFI_EVENT_QR_DISPLAY:
                ESP_LOGI(TAG, "Provisioning QR : %s", (char *)event_data);
                break;
            case APP_WIFI_EVENT_PROV_TIMEOUT:
                ESP_LOGI(TAG, "Provisioning Timed Out. Please reboot.");
                break;
            case APP_WIFI_EVENT_PROV_RESTART:
                ESP_LOGI(TAG, "Provisioning has restarted due to failures.");
                break;
            default:
                ESP_LOGW(TAG, "Unhandled App Wi-Fi Event: %"PRIi32, event_id);
                break;
        }
    } else if (event_base == RMAKER_OTA_EVENT) {
        switch(event_id) {
            case RMAKER_OTA_EVENT_STARTING:
                ESP_LOGI(TAG, "Starting OTA.");
                break;
            case RMAKER_OTA_EVENT_IN_PROGRESS:
                ESP_LOGI(TAG, "OTA is in progress.");
                break;
            case RMAKER_OTA_EVENT_SUCCESSFUL:
                ESP_LOGI(TAG, "OTA successful.");
                break;
            case RMAKER_OTA_EVENT_FAILED:
                ESP_LOGI(TAG, "OTA Failed.");
                break;
            case RMAKER_OTA_EVENT_REJECTED:
                ESP_LOGI(TAG, "OTA Rejected.");
                break;
            case RMAKER_OTA_EVENT_DELAYED:
                ESP_LOGI(TAG, "OTA Delayed.");
                break;
            case RMAKER_OTA_EVENT_REQ_FOR_REBOOT:
                ESP_LOGI(TAG, "Firmware image downloaded. Please reboot your device to apply the upgrade.");
                break;
            default:
                ESP_LOGW(TAG, "Unhandled OTA Event: %"PRIi32, event_id);
                break;
        }
    } else {
        ESP_LOGW(TAG, "Invalid event received!");
    }
}

void app_main(void)
{

   
    /* Initialize Application specific hardware drivers and
     * set initial state.
     */
    ledc_init();
    gpio_init();
    gpio_set_intr_type(SWITCH_PIN,  GPIO_INTR_POSEDGE);
    //gpio_set_intr_type(ENCODER_PIN, GPIO_INTR_NEGEDGE);
    gpio_evt_queue = xQueueCreate(1, sizeof(uint32_t));
    encoder_queue = xQueueCreate(1, sizeof(uint32_t));
    xTaskCreate(gpio_task, "gpio_task", 2048, NULL, 5, NULL); 
    xTaskCreate(encoder_task, "encoder_task", 2048, NULL, 10, NULL);  
    timer =  xTimerCreate("pinch_timer",
                            pdMS_TO_TICKS(500),
                            pdTRUE,
                            1,
                            pinch_timer);
    esp_rmaker_console_init();
    
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
    ESP_ERROR_CHECK(esp_event_handler_register(RMAKER_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(RMAKER_COMMON_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(APP_WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(RMAKER_OTA_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));

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
    
  
    //calibrate();

    
    
}
    