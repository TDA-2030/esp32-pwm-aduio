/* LED PWM PWM playing audio example

*/
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "driver/timer.h"
#include "soc/timer_group_struct.h"
#include "soc/ledc_struct.h"
#include "soc/ledc_reg.h"
#include "hal/gpio_ll.h"
#include "soc/timer_group_caps.h"
#include "sdkconfig.h"
#include "pwm_audio.h"
#ifdef CONFIG_IDF_TARGET_ESP32S2
#include "esp32s2/rom/ets_sys.h"
#endif /**< CONFIG_IDF_TARGET_ESP32S2 */
#ifdef CONFIG_IDF_TARGET_ESP32
#include "esp32/rom/ets_sys.h"
#endif /**< CONFIG_IDF_TARGET_ESP32 */



static const char *TAG = "pwm_audio";

#define PWM_AUDIO_CHECK(a, str, ret_val)                          \
    if (!(a))                                                     \
    {                                                             \
        ESP_LOGE(TAG, "%s(%d): %s", __FUNCTION__, __LINE__, str); \
        return (ret_val);                                         \
    }

static const char *PWM_AUDIO_PARAM_ADDR_ERROR = "PWM AUDIO PARAM ADDR ERROR";
static const char *PWM_AUDIO_PARAM_ERROR      = "PWM AUDIO PARAM ERROR";
static const char *PWM_AUDIO_FRAMERATE_ERROR  = "PWM AUDIO FRAMERATE ERROR";
static const char *PWM_AUDIO_STATUS_ERROR     = "PWM AUDIO STATUS ERROR";
static const char *PWM_AUDIO_TG_NUM_ERROR     = "PWM AUDIO TIMER GROUP NUMBER ERROR";
static const char *PWM_AUDIO_TIMER_NUM_ERROR  = "PWM AUDIO TIMER NUMBER ERROR";
static const char *PWM_AUDIO_ALLOC_ERROR      = "PWM AUDIO ALLOC ERROR";

#define BUFFER_MIN_SIZE (128UL)
#define SAMPLE_RATE_MAX (48000)
#define SAMPLE_RATE_MIN (8000)
#define CHANNEL_LEFT_INDEX  (0)
#define CHANNEL_RIGHT_INDEX (1)
#define CHANNEL_LEFT_MASK   (0x01)
#define CHANNEL_RIGHT_MASK  (0x02)


typedef struct {
    char *buf;                         /**< Original pointer */
    uint32_t volatile head;            /**< ending pointer */
    uint32_t volatile tail;            /**< Read pointer */
    uint32_t size;                     /**< Buffer size */

    SemaphoreHandle_t semaphore_rb;    /**< Semaphore for ringbuffer */

} ringBuf;
typedef ringBuf *ringbuf_handle_t;

typedef struct {
    pwm_audio_config_t    config;                          /**< pwm audio config struct */
    ledc_channel_config_t ledc_channel[PWM_AUDIO_CH_MAX];  /**< ledc channel config */
    ledc_timer_config_t   ledc_timer;                      /**< ledc timer config  */
    timg_dev_t            *timg_dev;                       /**< timer group register pointer */
    ringbuf_handle_t      ringbuf;                         /**< audio ringbuffer pointer */
    uint32_t              channel_mask;                    /**< channel gpio mask */
    uint32_t              channel_set_mask;                /**< channel set mask */

    pwm_audio_status_t status;
} pwm_audio_handle;
typedef pwm_audio_handle *pwm_audio_handle_t;

/**< pwm audio handle pointer */
static pwm_audio_handle_t g_pwm_audio_handle = NULL;

/**
 * Ringbuffer for pwm audio
 */
static esp_err_t rb_destroy(ringbuf_handle_t rb)
{
    if (rb == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (rb->buf) {
        free(rb->buf);
    }

    if (rb->semaphore_rb) {
        vSemaphoreDelete(rb->semaphore_rb);
    }

    free(rb);
    rb = NULL;
    return ESP_OK;
}
static ringbuf_handle_t rb_create(int size)
{
    if (size < (BUFFER_MIN_SIZE << 2)) {
        ESP_LOGE(TAG, "Invalid buffer size, Minimum = %d", (int32_t)(BUFFER_MIN_SIZE << 2));
        return NULL;
    }

    ringbuf_handle_t rb = NULL;
    char *buf = NULL;

    do {
        bool _success =
            (
                (rb             = malloc(sizeof(ringBuf))) &&
                (buf            = malloc(size))   &&
                (rb->semaphore_rb   = xSemaphoreCreateBinary())

            );

        if (!_success) {
            break;
        }

        rb->buf = buf;
        rb->head = rb->tail = 0;
        rb->size = size;
        return rb;

    } while (0);

    rb_destroy(rb);
    return NULL;
}

static uint32_t IRAM_ATTR rb_get_count(ringbuf_handle_t rb)
{
    uint32_t tail = rb->tail;

    if (rb->head >= tail) {
        return (rb->head - tail);
    }

    return (rb->size - (tail - rb->head));
}

static uint32_t IRAM_ATTR rb_get_free(ringbuf_handle_t rb)
{
    /** < Free a byte to judge the ringbuffer direction */
    return (rb->size - rb_get_count(rb) - 1);
}

static esp_err_t IRAM_ATTR rb_read_byte(ringbuf_handle_t rb, char *outdata)
{
    if (rb->tail == rb->head) {
        return ESP_FAIL;
    }

    // Send a byte from the buffer
    *outdata = rb->buf[rb->tail];

    // Update tail position
    rb->tail++;

    if (rb->tail == rb->size) {
        rb->tail = 0;
    }

    return ESP_OK;
}

static esp_err_t rb_write_byte(ringbuf_handle_t rb, const char indata)
{
    // Calculate next head
    uint32_t next_head = rb->head + 1;

    if (next_head == rb->size) {
        next_head = 0;
    }

    if (next_head == rb->tail) {
        return ESP_FAIL;
    }

    // Store data and advance head
    rb->buf[rb->head] = indata;
    rb->head = next_head;
    return ESP_OK;
}
static esp_err_t rb_write(ringbuf_handle_t rb, const char *inbuf, size_t len, size_t *bytes_written, TickType_t ticks_to_wait)
{
    if (xSemaphoreTake(rb->semaphore_rb, ticks_to_wait) == pdTRUE) {
        size_t free = rb_get_free(rb);
        size_t bytes_can_write = len;

        if (len > free) {
            bytes_can_write = free;
        }

        for (size_t i = 0; i < bytes_can_write; i++) {
            rb_write_byte(rb, inbuf[i]);
        }

        *bytes_written = bytes_can_write;
    }

    return ESP_OK;
}

/**
 * @brief Scale data to 16bit/32bit for pwm audio output.
 *        pwm audio can only output 8bit data value.
 */
static int audio_data_scale(int bits, uint8_t *sBuff, uint32_t len)
{
    if (bits == 16) {
        short *buf16 = (short *)sBuff;
        int k = len >> 1;

        for (int i = 0; i < k; i++) {
            buf16[i] &= 0xff00;
            buf16[i] += 0x8000;//turn signed value into unsigned, expand negative value into positive range
        }
    } else if (bits == 32) {
        int *buf32 = (int *)sBuff;
        int k = len >> 2;

        for (int i = 0; i < k; i++) {
            buf32[i] &= 0xff000000;
            buf32[i] += 0x80000000;//turn signed value into unsigned
        }
    } else {

        return 0;
    }

    return 0;
}
/*
 * Note:
 * In order to improve efficiency, register is operated directly
 */
static inline void ledc_set_duty_fast(ledc_mode_t speed_mode, ledc_channel_t channel_num, uint32_t duty_val)
{
    LEDC.channel_group[speed_mode].channel[channel_num].duty.duty = (duty_val) << 4; /* Discard decimal part */
    LEDC.channel_group[speed_mode].channel[channel_num].conf0.sig_out_en = 1;
    LEDC.channel_group[speed_mode].channel[channel_num].conf1.duty_start = 1;
    LEDC.channel_group[speed_mode].channel[channel_num].conf0.low_speed_update = 1;
}

/*
 * Timer group ISR handler
 */
static void IRAM_ATTR timer_group_isr(void *para)
{
    
    pwm_audio_handle_t handle = g_pwm_audio_handle;

    if (handle == NULL) {
        ets_printf("pwm audio not initialized\n");
        return;
    }

#ifdef CONFIG_IDF_TARGET_ESP32S2

    /* Clear the interrupt */
    if ((handle->timg_dev)->int_st.val & BIT(handle->config.timer_num)) {
        (handle->timg_dev)->int_clr.val |= (1UL << handle->config.timer_num);
    }

    /* After the alarm has been triggered
      we need enable it again, so it is triggered the next time */
    handle->timg_dev->hw_timer[handle->config.timer_num].config.alarm_en = TIMER_ALARM_EN;
#endif /**< CONFIG_IDF_TARGET_ESP32S2 */

#ifdef CONFIG_IDF_TARGET_ESP32

    /* Clear the interrupt */
    if (handle->timg_dev->int_st_timers.val & BIT(handle->config.timer_num)) {
        handle->timg_dev->int_clr_timers.val |= (1UL << handle->config.timer_num);
    }

    /* After the alarm has been triggered
      we need enable it again, so it is triggered the next time */
    handle->timg_dev->hw_timer[handle->config.timer_num].config.alarm_en = TIMER_ALARM_EN;

#endif /**< CONFIG_IDF_TARGET_ESP32 */

    char wave_h, wave_l;
    uint16_t value;
    /**< Output several channels when several pins are configured */

    if (handle->channel_mask & CHANNEL_LEFT_MASK) { /**< Determine whether the channel is configured */
        if (handle->config.bits_per_sample > 8) {
            rb_read_byte(handle->ringbuf, &wave_l);

            if (ESP_OK == rb_read_byte(handle->ringbuf, &wave_h)) {
                int16_t i = ((wave_h << 8) | wave_l);
                value = i + (1 << (handle->config.bits_per_sample - 1)); /**< offset */
                /**< set the PWM duty */
                ledc_set_duty_fast(handle->ledc_channel[CHANNEL_LEFT_INDEX].speed_mode, handle->ledc_channel[CHANNEL_LEFT_INDEX].channel, value);
            }
        } else {/**< spend 1.04us */GPIO.out_w1ts=0x8000;
            if (ESP_OK == rb_read_byte(handle->ringbuf, &wave_h)) {
                wave_h += 0x80; /**< offset */
                /**< set the PWM duty */
                ledc_set_duty_fast(handle->ledc_channel[CHANNEL_LEFT_INDEX].speed_mode, handle->ledc_channel[CHANNEL_LEFT_INDEX].channel, wave_h);
            }GPIO.out_w1tc=0x8000;
        }
    }

    if (handle->channel_mask & CHANNEL_RIGHT_MASK) { /**< Determine whether the channel is configured */
        if (handle->config.bits_per_sample > 8) {
            rb_read_byte(handle->ringbuf, &wave_l);

            if (ESP_OK == rb_read_byte(handle->ringbuf, &wave_h)) {
                int16_t i = ((wave_h << 8) | wave_l);
                value = i + (1 << (handle->config.bits_per_sample - 1)); /**< offset */
                /**< set the PWM duty */
                ledc_set_duty_fast(handle->ledc_channel[CHANNEL_LEFT_INDEX].speed_mode, handle->ledc_channel[CHANNEL_LEFT_INDEX].channel, value);
            }
        } else {
            if (ESP_OK == rb_read_byte(handle->ringbuf, &wave_h)) {
                wave_h += 0x80; /**< offset */
                /**< set the PWM duty */
                ledc_set_duty_fast(handle->ledc_channel[CHANNEL_LEFT_INDEX].speed_mode, handle->ledc_channel[CHANNEL_LEFT_INDEX].channel, wave_h);
            }
        }
    }

    /**
     * Send semaphore when buffer is less than minimum
     */
    if (rb_get_free(handle->ringbuf) > BUFFER_MIN_SIZE) {
        /**< spend 2.71us */
        BaseType_t xHigherPriorityTaskWoken;
        xSemaphoreGiveFromISR(handle->ringbuf->semaphore_rb, &xHigherPriorityTaskWoken);
        if (pdFALSE != xHigherPriorityTaskWoken) {
            portYIELD_FROM_ISR();
        }
    }
}

esp_err_t pwm_audio_get_status(pwm_audio_status_t *status)
{
    pwm_audio_handle_t handle = g_pwm_audio_handle;
    *status = handle->status;
    return ESP_OK;
}

esp_err_t pwm_audio_init(const pwm_audio_config_t *cfg)
{
    esp_err_t res = ESP_OK;
    PWM_AUDIO_CHECK(cfg != NULL, PWM_AUDIO_PARAM_ADDR_ERROR, ESP_ERR_INVALID_ARG);
    PWM_AUDIO_CHECK(cfg->tg_num < TIMER_GROUP_MAX, PWM_AUDIO_TG_NUM_ERROR, ESP_ERR_INVALID_ARG);
    PWM_AUDIO_CHECK(cfg->timer_num < TIMER_MAX, PWM_AUDIO_TIMER_NUM_ERROR, ESP_ERR_INVALID_ARG);

    pwm_audio_handle_t handle = NULL;

    handle = malloc(sizeof(pwm_audio_handle));
    PWM_AUDIO_CHECK(handle != NULL, PWM_AUDIO_ALLOC_ERROR, ESP_ERR_NO_MEM);
    memset(handle, 0, sizeof(pwm_audio_handle));

    handle->ringbuf = rb_create(cfg->ringbuf_len);
    PWM_AUDIO_CHECK(handle->ringbuf != NULL, PWM_AUDIO_ALLOC_ERROR, ESP_ERR_NO_MEM);

    handle->config = *cfg;
    g_pwm_audio_handle = handle;

    if (cfg->tg_num == TIMER_GROUP_0) {
        handle->timg_dev = &TIMERG0;
    } else {
        handle->timg_dev = &TIMERG1;
    }

    handle->channel_mask = 0;

    if (handle->config.gpio_num_left >= 0) {
        handle->ledc_channel[CHANNEL_LEFT_INDEX].channel = handle->config.ledc_channel_left;
        handle->ledc_channel[CHANNEL_LEFT_INDEX].duty = 0;
        handle->ledc_channel[CHANNEL_LEFT_INDEX].gpio_num = handle->config.gpio_num_left;
        handle->ledc_channel[CHANNEL_LEFT_INDEX].speed_mode = LEDC_LOW_SPEED_MODE;
        handle->ledc_channel[CHANNEL_LEFT_INDEX].hpoint = 0;
        handle->ledc_channel[CHANNEL_LEFT_INDEX].timer_sel = handle->config.ledc_timer_sel;
        handle->ledc_channel[CHANNEL_LEFT_INDEX].intr_type = LEDC_INTR_DISABLE;
        res = ledc_channel_config(&handle->ledc_channel[CHANNEL_LEFT_INDEX]);
        PWM_AUDIO_CHECK(ESP_OK == res, PWM_AUDIO_PARAM_ERROR, ESP_ERR_INVALID_ARG);
        handle->channel_mask |= CHANNEL_LEFT_MASK;
    }

    if (handle->config.gpio_num_right >= 0) {
        handle->ledc_channel[CHANNEL_RIGHT_INDEX].channel = handle->config.ledc_channel_right;
        handle->ledc_channel[CHANNEL_RIGHT_INDEX].duty = 0;
        handle->ledc_channel[CHANNEL_RIGHT_INDEX].gpio_num = handle->config.gpio_num_right;
        handle->ledc_channel[CHANNEL_RIGHT_INDEX].speed_mode = LEDC_LOW_SPEED_MODE;
        handle->ledc_channel[CHANNEL_RIGHT_INDEX].hpoint = 0;
        handle->ledc_channel[CHANNEL_RIGHT_INDEX].timer_sel = handle->config.ledc_timer_sel;
        handle->ledc_channel[CHANNEL_RIGHT_INDEX].intr_type = LEDC_INTR_DISABLE;
        res = ledc_channel_config(&handle->ledc_channel[CHANNEL_RIGHT_INDEX]);
        PWM_AUDIO_CHECK(ESP_OK == res, PWM_AUDIO_PARAM_ERROR, ESP_ERR_INVALID_ARG);
        handle->channel_mask |= CHANNEL_RIGHT_MASK;
    }

    PWM_AUDIO_CHECK(0 != handle->channel_mask, PWM_AUDIO_PARAM_ERROR, ESP_ERR_INVALID_ARG);

    res = pwm_audio_set_param(cfg->framerate, cfg->bits_per_sample, 2);
    PWM_AUDIO_CHECK(ESP_OK == res, PWM_AUDIO_PARAM_ERROR, ESP_ERR_INVALID_ARG);

    handle->status = PWM_AUDIO_STATUS_IDLE;

    return res;
}


esp_err_t pwm_audio_set_param(int rate, ledc_timer_bit_t bits, int ch)
{
    esp_err_t res;

    PWM_AUDIO_CHECK(g_pwm_audio_handle->status != PWM_AUDIO_STATUS_BUSY, PWM_AUDIO_STATUS_ERROR, ESP_ERR_INVALID_ARG);
    PWM_AUDIO_CHECK(rate <= SAMPLE_RATE_MAX && rate >= SAMPLE_RATE_MIN, PWM_AUDIO_FRAMERATE_ERROR, ESP_ERR_INVALID_ARG);
    PWM_AUDIO_CHECK(bits <= 12 && bits >= 8, PWM_AUDIO_FRAMERATE_ERROR, ESP_ERR_INVALID_ARG);
    PWM_AUDIO_CHECK(ch <= 2 && ch >= 1, PWM_AUDIO_FRAMERATE_ERROR, ESP_ERR_INVALID_ARG);

    pwm_audio_handle_t handle = g_pwm_audio_handle;

    handle->config.framerate = rate;
    handle->config.bits_per_sample = bits;
    handle->channel_set_mask = ch == 1 ? 0x00000001 : 0x00000003;

#ifdef CONFIG_IDF_TARGET_ESP32S2
    handle->ledc_timer.clk_cfg = LEDC_USE_APB_CLK;
#endif
    handle->ledc_timer.speed_mode = LEDC_LOW_SPEED_MODE;
    handle->ledc_timer.duty_resolution = handle->config.bits_per_sample;
    handle->ledc_timer.timer_num = handle->config.ledc_timer_sel;
    uint32_t freq = (APB_CLK_FREQ / (1 << handle->ledc_timer.duty_resolution));
    handle->ledc_timer.freq_hz = freq - (freq % 1000); // fixed PWM frequency ,It's a multiple of 1000
    res = ledc_timer_config(&handle->ledc_timer);
    PWM_AUDIO_CHECK(res == ESP_OK, PWM_AUDIO_PARAM_ERROR, ESP_ERR_INVALID_ARG);

    /* Select and initialize basic parameters of the timer */
    timer_config_t config = {0};
    config.divider = 16;
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en = TIMER_PAUSE;
    config.alarm_en = TIMER_ALARM_EN;
    config.intr_type = TIMER_INTR_LEVEL;
    config.auto_reload = 1;
#ifdef TIMER_GROUP_SUPPORTS_XTAL_CLOCK
    config.clk_src = TIMER_SRC_CLK_APB;  /* ESP32-S2 specific control bit !!!*/
#endif
    timer_init(handle->config.tg_num, handle->config.timer_num, &config);

    /* Timer's counter will initially start from value below.
    Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(handle->config.tg_num, handle->config.timer_num, 0x00000000ULL);

    /* Configure the alarm value and the interrupt on alarm. */
    timer_set_alarm_value(handle->config.tg_num, handle->config.timer_num, (TIMER_BASE_CLK / config.divider) / handle->config.framerate);
    timer_enable_intr(handle->config.tg_num, handle->config.timer_num);
    timer_isr_register(handle->config.tg_num, handle->config.timer_num, timer_group_isr, NULL, ESP_INTR_FLAG_IRAM, NULL);
    return ESP_OK;
}

esp_err_t pwm_audio_set_sample_rate(int rate)
{
    esp_err_t res;
    PWM_AUDIO_CHECK(g_pwm_audio_handle->status != PWM_AUDIO_STATUS_BUSY, PWM_AUDIO_STATUS_ERROR, ESP_ERR_INVALID_ARG);
    PWM_AUDIO_CHECK(rate <= SAMPLE_RATE_MAX && rate >= SAMPLE_RATE_MIN, PWM_AUDIO_FRAMERATE_ERROR, ESP_ERR_INVALID_ARG);

    pwm_audio_handle_t handle = g_pwm_audio_handle;
    handle->config.framerate = rate;
    uint16_t div = (uint16_t)handle->timg_dev->hw_timer[handle->config.timer_num].config.divider;
    res = timer_set_alarm_value(handle->config.tg_num, handle->config.timer_num, (TIMER_BASE_CLK / div) / handle->config.framerate);
    return res;
}

esp_err_t pwm_audio_write(char *inbuf, size_t len, size_t *bytes_written, TickType_t ticks_to_wait)
{
    esp_err_t res;
    pwm_audio_handle_t handle = g_pwm_audio_handle;
    PWM_AUDIO_CHECK(inbuf != NULL && bytes_written != NULL && len != 0, PWM_AUDIO_PARAM_ADDR_ERROR, ESP_ERR_INVALID_ARG);

    res = rb_write(handle->ringbuf, inbuf, len, bytes_written, ticks_to_wait);
    return res;
}

esp_err_t pwm_audio_start(void)
{
    esp_err_t res;
    pwm_audio_handle_t handle = g_pwm_audio_handle;
    PWM_AUDIO_CHECK(handle->status == PWM_AUDIO_STATUS_IDLE, PWM_AUDIO_STATUS_ERROR, ESP_ERR_INVALID_STATE);

    handle->status = PWM_AUDIO_STATUS_BUSY;
    timer_enable_intr(handle->config.tg_num, handle->config.timer_num);
    res = timer_start(handle->config.tg_num, handle->config.timer_num);
    return res;
}

esp_err_t pwm_audio_stop(void)
{
    pwm_audio_handle_t handle = g_pwm_audio_handle;
    timer_pause(handle->config.tg_num, handle->config.timer_num);
    timer_disable_intr(handle->config.tg_num, handle->config.timer_num);
    /**< just disable timer ,keep pwm output to reduce switching nosie */
    handle->status = PWM_AUDIO_STATUS_IDLE;
    return ESP_OK;
}

esp_err_t pwm_audio_deinit(void)
{
    pwm_audio_handle_t handle = g_pwm_audio_handle;
    PWM_AUDIO_CHECK(handle != NULL, PWM_AUDIO_PARAM_ADDR_ERROR, ESP_FAIL);

    handle->status = PWM_AUDIO_STATUS_UN_INIT;
    pwm_audio_stop();
    for (size_t i = 0; i < PWM_AUDIO_CH_MAX; i++) {
        if (handle->ledc_channel[i].gpio_num >= 0) {
            ledc_stop(handle->ledc_channel[i].speed_mode, handle->ledc_channel[i].channel, 0);
        }
    }
    for (size_t i = 0; i < PWM_AUDIO_CH_MAX; i++) {
        if (handle->ledc_channel[i].gpio_num >= 0) {
            gpio_set_direction(handle->ledc_channel[i].gpio_num, GPIO_MODE_INPUT);
        }
    }

    free(handle);
    rb_destroy(handle->ringbuf);
    return ESP_OK;
}
