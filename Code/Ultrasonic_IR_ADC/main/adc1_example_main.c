#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

#define DEFAULT_VREF    3300        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64         //Multisampling

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

void app_main()
{
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

    //Continuously sample ADC1
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
        
        distance_ir = (-31.06 * (double)voltage_ir * (double)voltage_ir * (double)voltage_ir) / (1000.0 * 1000.0 * 1000.0) + 
            (179.5 * (double)voltage_ir * (double)voltage_ir) / (1000.0 * 1000.0)  - (350.78 * (double)voltage_ir) / (1000.0) + 264.62; //ir conversion to cm
        
        distance_us = 0.1106 * (double)voltage_us + 13.619;  //ultrasonic conversion to cm
        distance_ir /= 100.0; //conversion to m
        distance_us /= 100.0;

        printf("Raw: %d\tVoltage: %dmV Distance_us: %fm Distance_ir: %fm\n", adc_reading_us, voltage_us, distance_us, distance_ir);
        //printf("Raw: %d\tVoltage: %dmV\n", adc_reading, voltage);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}