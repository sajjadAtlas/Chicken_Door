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
static void motor_stop();
static void motor_start(int direction);

static void pinch_timer(TimerHandle_t xTimer){
    printf("pinched\n");
    motor_stop();
    xTimerStop(xTimer, 0);
    vTaskDelay(10000/portTICK_PERIOD_MS);
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
            
            //cnt++;
            printf("pulse count: %ld\n", io_num);
            xTimerStart(timer, 0);
            if(io_num == 40){
                
                cnt = 0;
                gpio_set_intr_type(SWITCH_PIN,  GPIO_INTR_POSEDGE);
                //prevTime = 0;
                xTimerStop(timer, 0);
                motor_stop();
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

static void motor_start(int direction){

    if(direction == UP){

        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, 4096));
   
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0));
    }
   
    else if(direction == DOWN){

        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_1, LEDC_DUTY));
   
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_1));
    }
   
}

static void motor_stop(){

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

void app_main(void)
{

    
    gpio_init();
    gpio_set_intr_type(SWITCH_PIN,  GPIO_INTR_POSEDGE);
    //gpio_set_intr_type(ENCODER_PIN, GPIO_INTR_NEGEDGE);
    gpio_evt_queue = xQueueCreate(1, sizeof(uint32_t));
    encoder_queue = xQueueCreate(1, sizeof(uint32_t));
    xTaskCreate(gpio_task, "gpio_task", 2048, NULL, 5, NULL); 
    xTaskCreate(encoder_task, "encoder_task", 2048, NULL, 10, NULL);  
    ledc_init();
    timer =  xTimerCreate("pinch_timer",
                            pdMS_TO_TICKS(500),
                            pdTRUE,
                            1,
                            pinch_timer);
    calibrate();

    
    
}
    