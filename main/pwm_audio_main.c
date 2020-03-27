/* LEDC PWM Audio example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_err.h"

#include "pwm_audio.h"
#include "wave.h"

static const char *TAG = "pwm_audio";

#define REPEAT_PLAY

/* wave data */
char *wave_array;
uint32_t wave_size;
uint32_t wave_framerate;

#include "math.h"
#include "stdio.h"
#define PI_2 (6.283185307179f)
static void sin_test_task(void *arg)
{
    int8_t *buf;
    const uint32_t size = 240;
    uint32_t index = 0;
    size_t cnt;
    uint32_t block_w = 256;

    buf = malloc(size);
    if(buf == NULL)
    {
        ESP_LOGE(TAG, "malloc error");
        vTaskDelay(portMAX_DELAY);
    }
    for (size_t i = 0; i < size; i++)
    {
        buf[i]=127.8f*sinf(PI_2*(float)i/(float)size);
        printf("sin%d = %d\n", i, buf[i]);
    }

    pwm_audio_config_t pac;
    pac.framerate          = CONFIG_AUDIO_SAMPLE_RATE;
    pac.bits_per_sample    = LEDC_TIMER_8_BIT;
    pac.gpio_num_left      = CONFIG_LEFT_CHANNEL_GPIO;
    pac.ledc_channel_left  = LEDC_CHANNEL_0;
    pac.gpio_num_right     = -1;
    pac.ledc_channel_right = LEDC_CHANNEL_1;
    pac.ledc_timer_sel     = LEDC_TIMER_0;
    pac.tg_num             = TIMER_GROUP_0;
    pac.timer_num          = TIMER_0;
    pac.ringbuf_len        = 6000;
    pwm_audio_init(&pac);
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = 1<<15UL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
    pwm_audio_set_param(48000, 8, 2);
    pwm_audio_start();
    while (1)
    {
        if (index < size) {
            if ((size - index) < block_w) {
                block_w = size - index;
            }

            pwm_audio_write(( char *)buf + index, block_w, &cnt, 500);
            index += cnt;
        } else {

            index = 0;
            block_w = 256;

        }

        //vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}


static void pwm_audio_task(void *arg)
{
    wave_array     = wave_get();
    wave_size      = wave_get_size();
    wave_framerate = wave_get_framerate();

    pwm_audio_config_t pac;
    pac.framerate          = CONFIG_AUDIO_SAMPLE_RATE;
    pac.bits_per_sample    = LEDC_TIMER_8_BIT;
    pac.gpio_num_left      = CONFIG_LEFT_CHANNEL_GPIO;
    pac.ledc_channel_left  = LEDC_CHANNEL_0;
    pac.gpio_num_right     = CONFIG_RIGHT_CHANNEL_GPIO;
    pac.ledc_channel_right = LEDC_CHANNEL_1;
    pac.ledc_timer_sel     = LEDC_TIMER_0;
    pac.tg_num             = TIMER_GROUP_0;
    pac.timer_num          = TIMER_0;
    pac.ringbuf_len        = 6000;
    pwm_audio_init(&pac);
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = 1<<15UL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    uint32_t index = 0;
    size_t cnt;
    uint32_t block_w = 800;
    ESP_LOGI(TAG, "play init count");
    pwm_audio_set_param(wave_framerate, 8, 2);
    pwm_audio_start();

    while (1) {
        if (index < wave_size) {
            if ((wave_size - index) < block_w) {
                block_w = wave_size - index;
            }
            pwm_audio_write(( char *)wave_array + index, block_w, &cnt, 500);
            index += cnt;
        } else {

            ESP_LOGW(TAG, "play completed");
#ifdef REPEAT_PLAY
            index = 0;
            block_w = 800;
#else
            pwm_audio_stop();
            vTaskDelay(portMAX_DELAY);
#endif
        }

        //vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}
    

void app_main(void)
{
    ESP_LOGI(TAG, "----------start------------");

    // xTaskCreate(sin_test_task, "sin_test_task", 1024 * 3, NULL, 1, NULL);
    xTaskCreate(pwm_audio_task, "pwm_audio_task", 1024 * 2, NULL, 1, NULL);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    static char *task_info = NULL;
    task_info = (char *)malloc(1024);

    if (task_info == NULL) {
        ESP_LOGE(TAG, "malloc error");
        vTaskDelay(portMAX_DELAY);
    }

    /* Main loop */
    while (true) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        vTaskGetRunTimeStats(task_info);
        printf("\r\n------------\n%s", task_info);
        printf("the memory get: %d\n", esp_get_free_heap_size());
    }
}
