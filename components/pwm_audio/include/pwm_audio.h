
#ifndef _PWM_AUDIO_H_
#define _PWM_AUDIO_H_
#include "esp_err.h"
#include "driver/ledc.h"
#include "driver/timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief Configuration parameters of pwm audio for pwm_audio_init function
 */
typedef struct
{
    timer_group_t tg_num;             /*!< timer group number (0 - 1) */
    timer_idx_t timer_num;            /*!< timer number  (0 - 1) */
    int32_t framerate;                /*!< frame rates in Hz */
    ledc_timer_bit_t bits_per_sample; /*!< bits per sample (8 - 10) */

    int gpio_num_left;                  /*!< the LEDC output gpio_num, Left channel */
    int gpio_num_right;                  /*!< the LEDC output gpio_num, Right channel */
    ledc_channel_t ledc_channel_left;   /*!< LEDC channel (0 - 7), Corresponding to left channel*/
    ledc_channel_t ledc_channel_right;   /*!< LEDC channel (0 - 7), Corresponding to right channel*/
    ledc_timer_t ledc_timer_sel;   /*!< Select the timer source of channel (0 - 3) */

    uint32_t ringbuf_len;            /*!< ringbuffer size */

} pwm_audio_config_t;


/**
 * @brief pwm audio status
 */
typedef enum {
    PWM_AUDIO_STATUS_UN_INIT = 0, /*!< pwm audio uninitialized */
    PWM_AUDIO_STATUS_IDLE = 1, /*!< pwm audio idle */
    PWM_AUDIO_STATUS_BUSY = 2, /*!< pwm audio busy */
} pwm_audio_status_t;


/**
 * @brief pwm audio channel.
 *
 */
typedef enum {
    PWM_AUDIO_CH_MONO        = 0,            /*!< 1 channel (mono)*/
    PWM_AUDIO_CH_STEREO      = 1,            /*!< 2 channel (stereo)*/
    PWM_AUDIO_CH_MAX,
} pwm_audio_channel_t;


/**
 * @brief Initializes and configure the pwm audio.
 *        Configure pwm audio with the given source.
 * 
 * @param cfg Pointer of pwm_audio_config_t struct
 * 
 * @return
 *     - ESP_OK Success
 */
esp_err_t pwm_audio_init(const pwm_audio_config_t *cfg);


/**
 * @brief Start audio play
 * 
 * @return
 *     - ESP_OK Success
 */
esp_err_t pwm_audio_start(void);

/**
 * @brief Write data
 *
 * @param inbuf 
 * @param len 
 * @param bytes_written 
 * @param ticks_to_wait 
 * 
 * @return
 *     - ESP_OK Success
 */
esp_err_t pwm_audio_write(char *inbuf, size_t len, size_t *bytes_written, TickType_t ticks_to_wait);

/**
 * @brief stop audio play
 * 
 * @return
 *     - ESP_OK Success
 */
esp_err_t pwm_audio_stop(void);

/**
 * @brief Deinit pwm, timer and gpio
 *
 * @return
 *     - ESP_OK Success
 */
esp_err_t pwm_audio_deinit(void);

/**
 * @brief Set parameter for pwm audio.
 *
 * Similar to pwm_audio_set_sample_rate(), but also sets bit width.
 *
 * @param rate sample rate (ex: 8000, 44100...)
 *
 * @param bits bit width (MUST BE 8Bits)
 *
 * @param ch channel, (see pwm_audio_channel_t)
 *
 * @return
 *     - ESP_OK              Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t pwm_audio_set_param(int rate, ledc_timer_bit_t bits, int ch);

/**
 * @brief Set samplerate for pwm audio.
 *
 * @param rate sample rate (ex: 8000, 44100...)
 *
 * @return
 *     - ESP_OK              Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t pwm_audio_set_sample_rate(int rate);

/**
 * @brief get pwm audio status
 * 
 * @param status current pwm_audio status
 * 
 * @return
 *     - ESP_OK Success
 */
esp_err_t pwm_audio_get_status(pwm_audio_status_t *status);

#ifdef __cplusplus
}
#endif

#endif
