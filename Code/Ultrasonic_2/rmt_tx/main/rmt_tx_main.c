/* RMT transmit example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/rmt.h"
#include "driver/periph_ctrl.h"
#include "soc/rmt_reg.h"
#include "driver/gpio.h"
#include <time.h>


static const char *RMT_TX_TAG = "RMT Tx";

#define RMT_TX_CHANNEL 1
#define RMT_TX_GPIO 21

#define RMT_RX_CHANNEL 0
#define RMT_RX_GPIO_NUM 32
#define rmt_item32_tIMEOUT_US  9500   /*!< RMT receiver timeout value(us) */
#define RMT_CLK_DIV 100
#define RMT_TICK_10_US    (80000000/RMT_CLK_DIV/100000)   /*!< RMT counter value for 10 us.(Source clock is APB clock) */

#define SAMPLE_CNT  (10)

/*
 * Prepare a raw table with a message in the Morse code
 *
 * The message is "ESP" : . ... .--.
 *
 * The table structure represents the RMT item structure:
 * {duration, level, duration, level}
 *
 */
rmt_item32_t items[] = {
    // E : dot
    {{{ 32767, 1, 32767, 0 }}}, // dot
    //
    {{{ 32767, 0, 32767, 0 }}}, // SPACE
    // S : dot, dot, dot
    {{{ 32767, 1, 32767, 0 }}}, // dot
    {{{ 32767, 1, 32767, 0 }}}, // dot
    {{{ 32767, 1, 32767, 0 }}}, // dot
    //
    {{{ 32767, 0, 32767, 0 }}}, // SPACE
    // P : dot, dash, dash, dot
    {{{ 32767, 1, 32767, 0 }}}, // dot
    {{{ 32767, 1, 32767, 1 }}},
    {{{ 32767, 1, 32767, 0 }}}, // dash
    {{{ 32767, 1, 32767, 1 }}},
    {{{ 32767, 1, 32767, 0 }}}, // dash
    {{{ 32767, 1, 32767, 0 }}}, // dot

    // RMT end marker
    {{{ 0, 1, 0, 0 }}}
};

rmt_item32_t trig[1] = {
{{{ 10, 1 , 0 , 0 }}},
};

//Convert uint8_t type of data to rmt format data.
static void IRAM_ATTR u8_to_rmt(const void* src, rmt_item32_t* dest, size_t src_size, 
                         size_t wanted_num, size_t* translated_size, size_t* item_num)
{
    if(src == NULL || dest == NULL) {
        *translated_size = 0;
        *item_num = 0;
        return;
    }
    const rmt_item32_t bit0 = {{{ 32767, 1, 15000, 0 }}}; //Logical 0
    const rmt_item32_t bit1 = {{{ 32767, 1, 32767, 0 }}}; //Logical 1
    size_t size = 0;
    size_t num = 0;
    uint8_t *psrc = (uint8_t *)src;
    rmt_item32_t* pdest = dest;
    while (size < src_size && num < wanted_num) {
        for(int i = 0; i < 8; i++) {
            if(*psrc & (0x1 << i)) {
                pdest->val =  bit1.val; 
            } else {
                pdest->val =  bit0.val;
            }
            num++;
            pdest++;
        }
        size++;
        psrc++;
    }
    *translated_size = size;
    *item_num = num;
}

/*
 * Initialize the RMT Tx channel
 */
static void rmt_tx_int()
{
    rmt_config_t config;
    config.rmt_mode = RMT_MODE_TX;
    config.channel = RMT_TX_CHANNEL;
    config.gpio_num = RMT_TX_GPIO;
    config.mem_block_num = 1;
    config.tx_config.loop_en = 0;

    // enable the carrier to be able to hear the Morse sound
    // if the RMT_TX_GPIO is connected to a speaker
    config.tx_config.carrier_en = 0;
    config.tx_config.idle_output_en = 1;
    config.tx_config.idle_level = 0;
    config.tx_config.carrier_level = 1;

    // set the maximum clock divider to be able to output
    // RMT pulses in range of about one hundred milliseconds

    // CHANGE CLOCK DIVIDER TO 80 TO HAVE EACH UNIT TO BE APPROXIMATELY 1US.
    config.clk_div = 80; //255;

    rmt_config(&config);
    rmt_driver_install(config.channel, 0, 0);
}

static void rmt_rx_init()
{
    // Is the clock divider ok?
    rmt_config_t rmt_rx;
    rmt_rx.channel = RMT_RX_CHANNEL;
    rmt_rx.gpio_num = RMT_RX_GPIO_NUM;
    rmt_rx.clk_div = RMT_CLK_DIV; 
    rmt_rx.mem_block_num = 1;
    rmt_rx.rmt_mode = RMT_MODE_RX;
    rmt_rx.rx_config.filter_en = false;

    //rmt_rx.rx_config.filter_ticks_thresh = 50;
    //rmt_rx.rx_config.idle_threshold = rmt_item32_tIMEOUT_US / 10 * (RMT_TICK_10_US);
    rmt_config(&rmt_rx);
    rmt_driver_install(rmt_rx.channel, 1000, 0);
}

static void rmt_example_nec_rx_task()
{
	int channel = RMT_RX_CHANNEL;
    rmt_rx_init();
    RingbufHandle_t rb = NULL;
    //get RMT RX ringbuffer
    rmt_get_ringbuf_handle(channel, &rb);
    rmt_rx_start(channel, 1);
    while (rb) {
    	size_t rx_size = 0;
	    //try to receive data from ringbuffer.
	    //RMT driver will push all the data it receives to its ringbuffer.
	    rmt_item32_t* item = (rmt_item32_t*) xRingbufferReceive(rb, &rx_size, 1000);
	    // check time values of item.
	    if (item) {
	        int b0 = item[0].duration0; // RMT_TICK_10_US;
	        int b1 = item[0].level0;
	        int b2 = item[0].duration1; // RMT_TICK_10_US;
	        int b3 = item[0].level1;
	        printf("Buffered item: {%d, %d, %d, %d}\n", b0,b1,b2,b3);
	    }
    }
}


static void rmt_example_nec_tx_task()
{
    ESP_LOGI(RMT_TX_TAG, "Configuring transmitter");
    rmt_tx_int();
    // int number_of_items = sizeof(items) / sizeof(items[0]);
    // const uint8_t sample[SAMPLE_CNT] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};

    // start RX
    int channel = RMT_RX_CHANNEL;
    rmt_rx_init();
    RingbufHandle_t rb = NULL;
    //get RMT RX ringbuffer
    rmt_get_ringbuf_handle(channel, &rb);
    rmt_rx_start(channel, 1);

    while (1) {
    	size_t rx_size = 0;
        //ESP_ERROR_CHECK(rmt_write_items(RMT_TX_CHANNEL, items, number_of_items, true));
        rmt_write_items(RMT_TX_CHANNEL, trig, 1, true);
        //ESP_LOGI(RMT_TX_TAG, "Trigger signal transmitted.");

        //try to receive data from ringbuffer.
	    rmt_item32_t* item = (rmt_item32_t*) xRingbufferReceive(rb, &rx_size, 1000);;
	    
	    // check time values of item.
	    if (item != NULL) {
	        int b0 = item[0].duration0; // RMT_TICK_10_US;
	        int b1 = item[0].level0;
	        int b2 = item[0].duration1; // RMT_TICK_10_US;
	        int b3 = item[0].level1;
	        printf("Buffered item: {%d, %d, %d, %d}\n", b0,b1,b2,b3);

            // get distance from duration.

            float dist = ((float) item[0].duration0 ) / RMT_TICK_10_US // duration in tens of microseconds.
            dist = dist / 10.0 // duration in microseconds.
            dist = (dist * 0.034) / 2 // distance in cm.



	    }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        
    }
    vTaskDelete(NULL);
}

void app_main()
{
    xTaskCreate(rmt_example_nec_tx_task, "rmt_nec_tx_task", 2048, NULL, 10, NULL);
    //xTaskCreate(rmt_example_nec_rx_task, "rmt_nec_rx_task", 2048, NULL, 10, NULL);
}
