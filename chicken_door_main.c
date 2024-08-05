/* pwm example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
/*   DEPRECATED SINCE SWITCHING TO ESP32 AND MANY MANY MORE CHANGES, THIS IS SIMPLY FOR REFERENCE/DOCUMENTATION OF PROGRESS*/





   
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"

#include "esp8266/gpio_register.h"
#include "esp8266/pin_mux_register.h"

#include "driver/pwm.h"
#include "driver/gpio.h"
#include "driver/hw_timer.h"

#define GPIO_OUTPUT_IO_0    12
#define GPIO_INPUT_IO_0     4
#define GPIO_INPUT_IO_1     5
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_IO_0))
#define UP 0
#define DOWN 1
#define PWM_1_OUT_IO_NUM   15
#define PWM_2_OUT_IO_NUM   14

#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0)|(1ULL<<GPIO_INPUT_IO_1))

// PWM period 1000us(1Khz), same as depth
#define PWM_PERIOD    (1000)
gpio_config_t io_conf;
static const char *TAG = "pwm_example";
static const char *info = "INFO";
volatile int loopFlag = 1;
volatile uint32_t time = 0;
// pwm pin number
const uint32_t pin_num[2] = {
    PWM_1_OUT_IO_NUM,
    PWM_2_OUT_IO_NUM
    
};

// duties table, real_duty = duties[x]/PERIOD
uint32_t duties[2] = { 
    0, 0
};

// phase table, delay = (phase[x]/360)*PERIOD
float phase[2] = {
0, 90.0
    };
uint32_t *dutyVal = 0;
static xQueueHandle gpio_evt_queue = NULL;

static void gpio_isr_handler(void *arg)
{
    gpio_set_intr_type(GPIO_INPUT_IO_0, 0);
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
    gpio_set_intr_type(GPIO_INPUT_IO_0, 1);

}
volatile uint32_t cnter = 0;
volatile uint32_t switchState = 0;
volatile uint32_t target = 40;
volatile int switchFlag = 0;
volatile int motorState = 0;
volatile int calibrationMode = 0;
volatile int motorDirection = UP; 
volatile int prevTime = 0;
static void gpio_task_example(void *arg)
{
    uint32_t io_num;
   
    for (;;) {
        

        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
              

                
                if((!switchFlag || calibrationMode == 1)){
                    cnter++;
                    hw_timer_alarm_us(1000, 1);
                    ESP_LOGI(info, "encoderVal: %d\n", cnter);
                    //ESP_LOGI(info, "time diffxxx: %d\n", time - prevTime);
                    if(((time-prevTime)>320)){
                        ESP_LOGI(info, "PINCHED\n");
                        duties[0] = 0;
                        duties[1] = 0;
                        pwm_set_duties(duties);
                        pwm_start();
                        switchState = 0;
                            if(motorDirection == UP){
                                switchState = 0;
                                
                                loopFlag = 0;
                            }
                            else{  
                                vTaskDelay(3000 / portTICK_RATE_MS);
                                switchState = 1;
                            }
                        
                    }
                    prevTime = time;
                    
                }
              
                if(gpio_get_level(5)&&switchFlag==1){
                    switchState = 1;
                    motorState = 1;
                }
                
                if(cnter == target){
                   
                   
                    //ESP_LOGI(info, "elapsedTime: %d\n", elapsedTime);
                    pwm_stop(0x0);
                    duties[0] = 0;
                    duties[1] = 0;
                    pwm_set_duties(duties);
                    pwm_start();
                    gpio_set_intr_type(GPIO_INPUT_IO_0, 0);
                    switchState = 0;
                    loopFlag = 0;
                    cnter = 0;
                    time = 0;
                    prevTime = 0;
                }
                
               

            
           

        }
    }
    
     
}
void hw_timer_callback1(void *arg)
{
    time++;
}

void calibration(){

    duties[0] = 500;
    duties[1] = 0;
    calibrationMode = 1;
    pwm_set_duties(duties);
    pwm_start();
    
    ESP_LOGI(info, "CALIBRATION BEGINS\n");
    

}


void app_main()
{


    
    
    pwm_init(PWM_PERIOD, duties, 2, pin_num);
    pwm_set_phases(phase);
    duties[0] = 0;
    duties[1] = 0;
    pwm_set_duties(duties);
    pwm_start();

    

    
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO15/16
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;

    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_set_level(GPIO_OUTPUT_IO_0, 0);
    gpio_config(&io_conf);



    io_conf.intr_type = 1;
    gpio_set_intr_type(GPIO_INPUT_IO_1, 1);
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;

    io_conf.pull_up_en = 0;
    io_conf.pull_down_en = 0;
    gpio_config(&io_conf);
    gpio_evt_queue = xQueueCreate(1, sizeof(uint32_t));
    //start gpio task
    xTaskCreate(gpio_task_example, "gpio_task_example", 1024, NULL, 10 , NULL);

    //install gpio isr service
    gpio_install_isr_service(0);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void *) GPIO_INPUT_IO_0);
    gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler, (void *) GPIO_INPUT_IO_1);

    


 
    gpio_set_level(GPIO_OUTPUT_IO_0, 1);

    hw_timer_init(hw_timer_callback1, NULL);
    calibration();
    ESP_LOGI(TAG, "MOTOR RUNNING\n");
    while (1) {
        
       
       
        if(!motorState){
            
            if(calibrationMode == 0){
                gpio_set_intr_type(GPIO_INPUT_IO_0, 0);
            }
            gpio_set_intr_type(GPIO_INPUT_IO_1, 1);
        
            switchFlag = 1;   

            
            
           
        } 
        else if (switchState) {
            gpio_set_intr_type(GPIO_INPUT_IO_1, 0);
            motorState = 1;
            switchFlag = 0;
            
            if(motorDirection == UP){
                duties[0] = 500;
                duties[1] = 0;
                
            }
            else if (motorDirection == DOWN){
                duties[0] = 0;
                duties[1] = 500;
            }   
            pwm_set_duties(duties);
            pwm_start();
            gpio_set_intr_type(GPIO_INPUT_IO_0, 1);
            
            
        } 
        else if(loopFlag==0) {
            switchState = 0;
            motorState = 0;
            cnter = 0;
            //ESP_LOGI(TAG, "PWM STOP [2]\n");
            
            motorDirection = !motorDirection;
            
            if(calibrationMode){
                motorDirection = DOWN;
                calibrationMode = 0;
               
            }
            
            ESP_LOGI(TAG, "MOTOR STOPPED\n");
            gpio_set_intr_type(GPIO_INPUT_IO_1, 1);
            switchFlag = 1;
            loopFlag = 1;
            time = 0;
            prevTime = 0;
            hw_timer_disarm();
            
        }
        
    
        
       
        //printf("COUNT VALUE: %d\n", count);
       
       vTaskDelay(1);

    }
}
