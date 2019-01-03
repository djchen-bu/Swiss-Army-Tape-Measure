#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/uart.h"
#include "esp_adc_cal.h"

#include "esp_types.h"
#include "freertos/queue.h"
#include "soc/timer_group_struct.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"

#define DEFAULT_VREF    3300        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64         //Multisampling

//lidar defines
#define ECHO_TEST_TXD   17
#define ECHO_TEST_RXD   16
#define ECHO_TEST_RTS  (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS  (UART_PIN_NO_CHANGE)

#define BUF_SIZE (1024)

//wheelspeed
#define GPIO_SPEED 25

#define TIMER_DIVIDER         16  //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds
#define TIMER_INTERVAL_SEC   (1.0) // sample test interval for the first timer
#define TEST_WITHOUT_RELOAD   0
#define TEST_WITH_RELOAD      1

#define PI 3.14159
#define WHEEL_DIAMETER 2.5 //in centimeters


static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_6;     //GPIO34 if ADC1, GPIO14 if ADC2
static const adc_atten_t atten_ir = ADC_ATTEN_DB_11; //ir sensor
static const adc_atten_t atten_us = ADC_ATTEN_DB_0; //ultrasonic 1
static const adc_unit_t unit = ADC_UNIT_1;
static const adc_unit_t unit1 = ADC_UNIT_2;

static void check_efuse()
{
    //Check TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }

    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
}

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}

static void example_tg0_timer_init(int timer_idx, 
    bool auto_reload, double timer_interval_sec)
{
    /* Select and initialize basic parameters of the timer */
    timer_config_t config;
    config.divider = TIMER_DIVIDER;
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en = TIMER_PAUSE;
    config.alarm_en = TIMER_ALARM_DIS;
    config.intr_type = TIMER_INTR_LEVEL;
    config.auto_reload = auto_reload;
    timer_init(TIMER_GROUP_0, timer_idx, &config);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(TIMER_GROUP_0, timer_idx, 0x00000000ULL);
}

static void sensor_data(){
    //Check if Two Point or Vref are burned into eFuse
    check_efuse();

    //Configure ADC
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(channel, atten_us);
    adc2_config_channel_atten((adc2_channel_t)channel, atten_ir);

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type_us = esp_adc_cal_characterize(unit, atten_us, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    esp_adc_cal_value_t val_type_ir = esp_adc_cal_characterize(unit1, atten_ir, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type_us);
    print_char_val_type(val_type_ir);

    double distance_us;
    double distance_ir;
    double distance_lidar;

    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS);
    uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);
    
    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    uint16_t distance; 
    int length;

    while (1) {
        uint32_t adc_reading_us = 0;
        uint32_t adc_reading_ir = 0;
        //Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            adc_reading_us += adc1_get_raw((adc1_channel_t)channel);
            int raw;
            adc2_get_raw((adc2_channel_t)channel, ADC_WIDTH_BIT_12, &raw);
            adc_reading_ir += raw;
        }
        adc_reading_us /= NO_OF_SAMPLES;
        adc_reading_ir /= NO_OF_SAMPLES;
        //Convert adc_reading to voltage in mV
        uint32_t voltage_us = esp_adc_cal_raw_to_voltage(adc_reading_us, adc_chars);
        uint32_t voltage_ir = esp_adc_cal_raw_to_voltage(adc_reading_ir, adc_chars);
        
        length = 0;
        uart_get_buffered_data_len(UART_NUM_1,(size_t*)&length);
       
        int len = uart_read_bytes(UART_NUM_1,data,length,100);

        distance = (data[3]<<8)+data[2];
        distance_ir = (-31.06 * (double)voltage_ir * (double)voltage_ir * (double)voltage_ir) / (1000.0 * 1000.0 * 1000.0) + 
            (179.5 * (double)voltage_ir * (double)voltage_ir) / (1000.0 * 1000.0)  - (350.78 * (double)voltage_ir) / (1000.0) + 264.62; //ir conversion to cm

        distance_us = 0.1106 * (double)voltage_us + 13.619;  //ultrasonic conversion to cm
        distance_ir /= 100.0; //conversion to m
        distance_us /= 100.0;
        distance_lidar = (double)distance / 100.0;

        printf("Ultrasonic %f\n", distance_us);
        printf("IR %f\n", distance_ir);
        printf("Lidar %f\n", distance_lidar);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void wheel_speed(){
    example_tg0_timer_init(TIMER_0, TEST_WITH_RELOAD, TIMER_INTERVAL_SEC);

    int analog;
    double count;
    double oldcount = 0;
    double speed = 0;

    gpio_pad_select_gpio(GPIO_SPEED);
    gpio_set_direction(GPIO_SPEED, GPIO_MODE_INPUT);

    timer_start(TIMER_GROUP_0, TIMER_0);
    while(1){
        analog = gpio_get_level(GPIO_SPEED);
        vTaskDelay(10 / portTICK_PERIOD_MS);
        if(!analog && gpio_get_level(GPIO_SPEED)){
            timer_get_counter_time_sec(TIMER_GROUP_0, TIMER_0, &count);
            speed = PI * WHEEL_DIAMETER / 100.0 / (count - oldcount);
            oldcount = count;
            printf("Wheel %f\n", speed);
        }
    }
}

void app_main()
{
    xTaskCreate(sensor_data, "sensor_data", 4096, NULL, 5, NULL);
    xTaskCreate(wheel_speed, "wheel_speed", 4096, NULL, 5, NULL);
}