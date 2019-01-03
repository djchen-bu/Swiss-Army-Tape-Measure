/* brushed dc motor control example
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/*
 * This example will show you how to use MCPWM module to control brushed dc motor.
 * This code is tested with L298 motor driver.
 * User may need to make changes according to the motor driver they use.
*/

#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

#define GPIO_PWM0A_OUT 14   //LEFT Set GPIO 15 as PWM0A
#define GPIO_PWM1A_OUT 32   //RIGHT Set GPIO 16 as PWM0B

// GPIO SETS DIRECTIONALITY:
#define L1 15
#define L2 33

#define R3 27
#define R4 12

#define GPIO_SEL  ((1ULL<<L1) | (1ULL<<L2) | (1ULL<<R3) | (1ULL<<R4))

static void mcpwm_example_gpio_initialize()
{
    printf("initializing mcpwm gpio...\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT); // LEFT
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1A, GPIO_PWM1A_OUT); // RIGHT

    // INITIALIZE GPIO FOR DIRECTIONALITY;
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set
    io_conf.pin_bit_mask = GPIO_SEL;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

}

/**
 * @brief motor moves in forward direction, with duty cycle = duty %
 */

static void brushed_motor_forward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num, float duty_cycle)
{
    // set directionality
    gpio_set_level(L1, 1);
    gpio_set_level(L2, 0);

    gpio_set_level(R3, 1);
    gpio_set_level(R4, 0);

    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_B);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, duty_cycle);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state

}

/**
 * @brief motor moves in backward direction, with duty cycle = duty %
 */

static void brushed_motor_backward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num, float duty_cycle)
{
    // set directionality
    gpio_set_level(L1, 0);
    gpio_set_level(L2, 1);

    gpio_set_level(R3, 0);
    gpio_set_level(R4, 1);
    /*

    Why is the below commented out? 
    ANS: Experimentally, I found that the alternating between OPR_A and OPR_B in each pwm pin was accomplishing nothing.
    This is due to the fact that we are not changing the pulse emitted, rather using the GPIO pins to change directionality.

    mcpwm_set_signal_low(mcpwm_num0, timer_num0, MCPWM_OPR_A);
    mcpwm_set_duty(mcpwm_num0, timer_num0, MCPWM_OPR_B, duty_cycle);
    mcpwm_set_duty_type(mcpwm_num0, timer_num0, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);  //call this each time, if operator was previously in low/high state

    */
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_B);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, duty_cycle);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
    
}

/**
 * @brief motor stop
 */
static void brushed_motor_stop(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num)
{
    gpio_set_level(L1, 0);
    gpio_set_level(L2, 0);

    gpio_set_level(R3, 0);
    gpio_set_level(R4, 0);

    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_A);
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_B);
}

/**
 * @brief Configure MCPWM module for brushed dc motor
 */
static void mcpwm_example_brushed_motor_control(void *arg)
{
    //1. mcpwm gpio initialization
    mcpwm_example_gpio_initialize();

    //2. initial mcpwm configuration
    printf("Configuring Initial Parameters of mcpwm...\n");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 1000;    //frequency = 500Hz,
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &pwm_config);
    

    // fwd left motor
    brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 90.0);
	vTaskDelay(500 / portTICK_RATE_MS);
	// fwd right motor
	brushed_motor_forward(MCPWM_UNIT_1, MCPWM_TIMER_1, 90.0);
	vTaskDelay(500 / portTICK_RATE_MS);


	// rev left motor
	brushed_motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_0, 90.0);
	vTaskDelay(500 / portTICK_RATE_MS);
    // rev right motor
    brushed_motor_backward(MCPWM_UNIT_1, MCPWM_TIMER_1, 90.0);
	vTaskDelay(500 / portTICK_RATE_MS);

    // run the motor through different pulse widths
    while (1) {
        float p = 40.0;
        for (int i = 0; i < 7; i++) {
            p = p + 10;
            brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, p);
            brushed_motor_forward(MCPWM_UNIT_1, MCPWM_TIMER_1, p);
            vTaskDelay(1000 / portTICK_RATE_MS);
        }
    }

    while (1) {

    	// fwd both
    	vTaskDelay(1000 / portTICK_RATE_MS);
        brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 90.0);
        brushed_motor_forward(MCPWM_UNIT_1, MCPWM_TIMER_1, 90.0);
        vTaskDelay(1000 / portTICK_RATE_MS);

        brushed_motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
        brushed_motor_stop(MCPWM_UNIT_1, MCPWM_TIMER_1);
        vTaskDelay(2000 / portTICK_RATE_MS);

        // rev both
        brushed_motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_0, 90.0);
        brushed_motor_backward(MCPWM_UNIT_1, MCPWM_TIMER_1, 90.0);
        vTaskDelay(1000 / portTICK_RATE_MS);

        brushed_motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
        brushed_motor_stop(MCPWM_UNIT_1, MCPWM_TIMER_1);
        vTaskDelay(2000 / portTICK_RATE_MS);
    }
}

void app_main()
{
    printf("Testing brushed motor...\n");
    xTaskCreate(mcpwm_example_brushed_motor_control, "mcpwm_examlpe_brushed_motor_control", 4096, NULL, 5, NULL);
}