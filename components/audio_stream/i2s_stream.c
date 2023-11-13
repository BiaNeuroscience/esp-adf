/*
 * ESPRESSIF MIT License
 *
 * Copyright (c) 2018 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on all ESPRESSIF SYSTEMS products, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "driver/i2s.h"
#include "soc/io_mux_reg.h"
#include "esp_log.h"
#include "esp_err.h"

#include "audio_common.h"
#include "audio_mem.h"
#include "audio_element.h"
#include "i2s_stream.h"
#include "esp_alc.h"
#include "board_pins_config.h"
#include "audio_idf_version.h"

static const char *TAG = "I2S_STREAM";

#define I2S_BUFFER_ALINED_BYTES_SIZE (12)

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(4, 2, 0)
#define SOC_I2S_SUPPORTS_ADC_DAC 1
#elif (SOC_DAC_SUPPORTED)
#if CONFIG_IDF_TARGET_ESP32
#define SOC_I2S_SUPPORTS_ADC_DAC 1
#endif
#endif
typedef struct i2s_stream {
    audio_stream_type_t type;
    i2s_stream_cfg_t    config;
    bool                is_open;
    bool                use_alc;
    void               *volume_handle;
    int                 volume;
    bool                uninstall_drv;
    int                 data_bit_width;
    int                 target_bits;

#if (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0))
    i2s_chan_handle_t   rx_handle;
    i2s_chan_handle_t   tx_handle;
#endif
} i2s_stream_t;

#if (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0))
static struct i2s_bus_s {
    int  ref_count;
    i2s_chan_handle_t   rx_handle;
    i2s_chan_handle_t   tx_handle;
} s_i2s_bus = { 0 };
#endif

static int i2s_driver_startup(audio_element_handle_t self, i2s_stream_cfg_t *i2s_cfg, i2s_stream_t *i2s);
static int i2s_driver_cleanup(i2s_stream_t *i2s);
static int i2s_common_read(i2s_stream_t *i2s, char *buffer, size_t size,  TickType_t ticks_to_wait);
static int i2s_common_write(i2s_stream_t *i2s, char *buffer, size_t size,  TickType_t ticks_to_wait);
static esp_err_t i2s_common_set_clk(i2s_stream_t *i2s, int rate, int bits, int ch);

#ifdef SOC_I2S_SUPPORTS_ADC_DAC
static esp_err_t i2s_mono_fix(int bits, uint8_t *sbuff, uint32_t len)
{
    if (bits == 16) {
        int16_t *temp_buf = (int16_t *)sbuff;
        int16_t temp_box;
        int k = len >> 1;
        for (int i = 0; i < k; i += 2) {
            temp_box = temp_buf[i];
            temp_buf[i] = temp_buf[i + 1];
            temp_buf[i + 1] = temp_box;
        }
    } else if (bits == 32) {
        int32_t *temp_buf = (int32_t *)sbuff;
        int32_t temp_box;
        int k = len >> 2;
        for (int i = 0; i < k; i += 4) {
            temp_box = temp_buf[i];
            temp_buf[i] = temp_buf[i + 1];
            temp_buf[i + 1] = temp_box;
        }
    } else {
        ESP_LOGE(TAG, "%s %dbits is not supported", __func__, bits);
        return ESP_FAIL;
    }
    return ESP_OK;
}

/**
 * @brief Scale data to 16bit/32bit for I2S DMA output.
 *        DAC can only output 8bit data value.
 *        I2S DMA will still send 16bit or 32bit data, the highest 8bit contains DAC data.
 */
static int i2s_dac_data_scale(int bits, uint8_t *sBuff, uint32_t len)
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
        ESP_LOGE(TAG, "in %s %dbits is not supported", __func__, bits);
        return -1;
    }

    return 0;
}
#endif

static inline esp_err_t i2s_stream_check_data_bits(i2s_stream_t *i2s, int bits)
{
#if CONFIG_IDF_TARGET_ESP32
    if (bits == I2S_BITS_PER_SAMPLE_24BIT) {
        i2s->config.expand_src_bits = bits;
        i2s->data_bit_width = I2S_BITS_PER_SAMPLE_24BIT;
    } else {
        i2s->data_bit_width = 0;
    }
#endif
    return ESP_OK;
}

static esp_err_t _i2s_open(audio_element_handle_t self)
{
    i2s_stream_t *i2s = (i2s_stream_t *)audio_element_getdata(self);
    ESP_LOGE(TAG, "_i2s_open");
    if (i2s->is_open) {
        return ESP_OK;
    }

    if (i2s->type == AUDIO_STREAM_WRITER) {
        audio_element_set_input_timeout(self, 10 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "AUDIO_STREAM_WRITER");
    }
    if (i2s->use_alc) {
        i2s->volume_handle = alc_volume_setup_open();
        if (i2s->volume_handle == NULL) {
            ESP_LOGE(TAG, "i2s create the handle for setting volume failed, in line(%d)", __LINE__);
            return ESP_FAIL;
        }
    }
    i2s->is_open = true;

    return ESP_OK;
}

static esp_err_t _i2s_destroy(audio_element_handle_t self)
{
    i2s_stream_t *i2s = (i2s_stream_t *)audio_element_getdata(self);
    if (i2s->uninstall_drv) {
        i2s_driver_cleanup(i2s);
    }
    audio_free(i2s);
    return ESP_OK;
}

static esp_err_t _i2s_close(audio_element_handle_t self)
{
    i2s_stream_t *i2s = (i2s_stream_t *)audio_element_getdata(self);
    i2s->is_open = false;
    if (AEL_STATE_PAUSED != audio_element_get_state(self)) {
        audio_element_report_pos(self);
        audio_element_set_byte_pos(self, 0);
    }
    if (i2s->use_alc) {
        if (i2s->volume_handle != NULL) {
            alc_volume_setup_close(i2s->volume_handle);
        }
    }
    return ESP_OK;
}

static int _i2s_read(audio_element_handle_t self, char *buffer, int len, TickType_t ticks_to_wait, void *context)
{
    i2s_stream_t *i2s = (i2s_stream_t *)audio_element_getdata(self);
    size_t bytes_read = 0;
    bytes_read = i2s_common_read(i2s, buffer, len, ticks_to_wait);
    ESP_LOG_BUFFER_HEX(TAG, buffer, 32);
    audio_element_info_t info;
    audio_element_getinfo(self, &info);
    if (bytes_read > 0) {
#if CONFIG_IDF_TARGET_ESP32
        if (info.channels == 1) {
            i2s_mono_fix(info.bits, (uint8_t *)buffer, bytes_read);
        }
#endif
    }
    return bytes_read;
}

static int _i2s_write(audio_element_handle_t self, char *buffer, int len, TickType_t ticks_to_wait, void *context)
{
    i2s_stream_t *i2s = (i2s_stream_t *)audio_element_getdata(self);
    size_t bytes_written = 0;
    audio_element_info_t info;
    audio_element_getinfo(self, &info);
    i2s->target_bits = info.bits;
    if (len > 0) {
#if CONFIG_IDF_TARGET_ESP32
        target_bits = I2S_BITS_PER_SAMPLE_32BIT;
        if (info.channels == 1) {
            i2s_mono_fix(info.bits, (uint8_t *)buffer, len);
        }
#endif
#if SOC_I2S_SUPPORTS_ADC_DAC
        if ((i2s->config.i2s_config.mode & I2S_MODE_DAC_BUILT_IN) != 0) {
            i2s_dac_data_scale(info.bits, (uint8_t *)buffer, len);
        }
#endif
    }

    if (len) {
        if ((i2s->config.need_expand && (i2s->target_bits != i2s->config.expand_src_bits)) || (i2s->data_bit_width == I2S_BITS_PER_SAMPLE_24BIT)) {
            i2s->config.need_expand = true;
        } else {
            i2s->config.need_expand = false;
        }
        bytes_written = i2s_common_write(i2s, buffer, len, ticks_to_wait);
        if (bytes_written) {
            audio_element_update_byte_pos(self, bytes_written);
        }
    }
    return bytes_written;
}

static int _i2s_process(audio_element_handle_t self, char *in_buffer, int in_len)
{
    int r_size = audio_element_input(self, in_buffer, in_len);
    int w_size = 0;
    i2s_stream_t *i2s = (i2s_stream_t *)audio_element_getdata(self);
    audio_element_info_t i2s_info = {0};
    if (r_size == AEL_IO_TIMEOUT) {
#if SOC_I2S_SUPPORTS_ADC_DAC
        if ((i2s->config.i2s_config.mode & I2S_MODE_DAC_BUILT_IN) != 0) {
            memset(in_buffer, 0x80, in_len);
        } else
#endif
        {
            memset(in_buffer, 0x00, in_len);
        }
        r_size = in_len;
        audio_element_multi_output(self, in_buffer, r_size, 0);
        w_size = audio_element_output(self, in_buffer, r_size);
    } else if (r_size > 0) {
        if (i2s->use_alc) {
            audio_element_getinfo(self, &i2s_info);
            alc_volume_setup_process(in_buffer, r_size, i2s_info.channels, i2s->volume_handle, i2s->volume);
        }
        audio_element_multi_output(self, in_buffer, r_size, 0);
        w_size = audio_element_output(self, in_buffer, r_size);
        audio_element_update_byte_pos(self, w_size);
    } else {
        w_size = r_size;
    }
    return w_size;
}

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(4, 4, 0)
#if CONFIG_IDF_TARGET_ESP32
static esp_err_t i2s_mclk_gpio_select(i2s_port_t i2s_num, gpio_num_t gpio_num)
{
    if (i2s_num >= SOC_I2S_NUM) {
        return ESP_ERR_INVALID_ARG;
    }
    if (i2s_num == I2S_NUM_0) {
        if (gpio_num == GPIO_NUM_0) {
            PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0_CLK_OUT1);
            WRITE_PERI_REG(PIN_CTRL, 0xFFF0);
        } else if (gpio_num == GPIO_NUM_1) {
            PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0TXD_U, FUNC_U0TXD_CLK_OUT3);
            WRITE_PERI_REG(PIN_CTRL, 0xF0F0);
        } else {
            PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0RXD_U, FUNC_U0RXD_CLK_OUT2);
            WRITE_PERI_REG(PIN_CTRL, 0xFF00);
        }
    } else if (i2s_num == I2S_NUM_1) {
        if (gpio_num == GPIO_NUM_0) {
            PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0_CLK_OUT1);
            WRITE_PERI_REG(PIN_CTRL, 0xFFFF);
        } else if (gpio_num == GPIO_NUM_1) {
            PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0TXD_U, FUNC_U0TXD_CLK_OUT3);
            WRITE_PERI_REG(PIN_CTRL, 0xF0FF);
        } else {
            PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0RXD_U, FUNC_U0RXD_CLK_OUT2);
            WRITE_PERI_REG(PIN_CTRL, 0xFF0F);
        }
    }
    return ESP_OK;
}
#else
static esp_err_t i2s_mclk_gpio_select(i2s_port_t i2s_num, gpio_num_t gpio_num)
{
    return ESP_FAIL;
}
#endif
#endif

esp_err_t i2s_stream_set_clk(audio_element_handle_t i2s_stream, int rate, int bits, int ch)
{
    esp_err_t err = ESP_OK;
    i2s_stream_t *i2s = (i2s_stream_t *)audio_element_getdata(i2s_stream);
    audio_element_state_t state = audio_element_get_state(i2s_stream);
    if (state == AEL_STATE_RUNNING) {
        audio_element_pause(i2s_stream);
    }
    audio_element_set_music_info(i2s_stream, rate, ch, bits);
    i2s_stream_check_data_bits(i2s, bits);

    i2s_common_set_clk(i2s, rate, bits, ch);
    if (state == AEL_STATE_RUNNING) {
        audio_element_resume(i2s_stream, 0, 0);
    }
    return err;
}

int i2s_alc_volume_set(audio_element_handle_t i2s_stream, int volume)
{
    i2s_stream_t *i2s = (i2s_stream_t *)audio_element_getdata(i2s_stream);
    if (i2s->use_alc) {
        i2s->volume = volume;
        return ESP_OK;
    } else {
        ESP_LOGW(TAG, "The ALC don't be used. It can not be set.");
        return ESP_FAIL;
    }
}

int i2s_alc_volume_get(audio_element_handle_t i2s_stream, int *volume)
{
    i2s_stream_t *i2s = (i2s_stream_t *)audio_element_getdata(i2s_stream);
    if (i2s->use_alc) {
        *volume = i2s->volume;
        return ESP_OK;
    } else {
        ESP_LOGW(TAG, "The ALC don't be used");
        return ESP_FAIL;
    }
}

audio_element_handle_t i2s_stream_init(i2s_stream_cfg_t *config)
{
    audio_element_cfg_t cfg = DEFAULT_AUDIO_ELEMENT_CONFIG();
    audio_element_handle_t el;
    cfg.open = _i2s_open;
    cfg.close = _i2s_close;
    cfg.process = _i2s_process;
    cfg.destroy = _i2s_destroy;
    cfg.task_stack = config->task_stack;
    cfg.task_prio = config->task_prio;
    cfg.task_core = config->task_core;
    cfg.stack_in_ext = config->stack_in_ext;
    cfg.out_rb_size = config->out_rb_size;
    cfg.multi_out_rb_num = config->multi_out_num;
    cfg.tag = "iis";
    cfg.buffer_len = config->buffer_len;

    if (cfg.buffer_len % I2S_BUFFER_ALINED_BYTES_SIZE) {
        ESP_LOGE(TAG, "The size of buffer must be a multiple of %d, current size is %d", I2S_BUFFER_ALINED_BYTES_SIZE, cfg.buffer_len);
        return NULL;
    }

    i2s_stream_t *i2s = audio_calloc(1, sizeof(i2s_stream_t));
    AUDIO_MEM_CHECK(TAG, i2s, return NULL);
    memcpy(&i2s->config, config, sizeof(i2s_stream_cfg_t));

    i2s->type = config->type;
    i2s->use_alc = config->use_alc;
    i2s->volume = config->volume;
    i2s->uninstall_drv = config->uninstall_drv;
    if (config->type == AUDIO_STREAM_READER) {
        cfg.read = _i2s_read;
    } else if (config->type == AUDIO_STREAM_WRITER) {
        cfg.write = _i2s_write;
    }
    // i2s_stream_check_data_bits(i2s, i2s->config.i2s_config.bits_per_sample);
    // i2s_stream_check_data_bits(i2s, i2s->config.transmit_cfg.std_mode.std_config.clk_cfg.sample_rate_hz);

    el = audio_element_init(&cfg);
    AUDIO_MEM_CHECK(TAG, el, {
        audio_free(i2s);
        return NULL;
    });
    audio_element_setdata(el, i2s);

    if (i2s_driver_startup(el, config, i2s) != ESP_OK) {
        audio_free(i2s);
        return NULL;
    }

    return el;
}

esp_err_t i2s_stream_sync_delay(audio_element_handle_t i2s_stream, int delay_ms)
{
    char *in_buffer = NULL;

    audio_element_info_t info;
    audio_element_getinfo(i2s_stream, &info);

    if (delay_ms < 0) {
        uint32_t delay_size = (~delay_ms + 1) * ((uint32_t)(info.sample_rates * info.channels * info.bits / 8) / 1000);
        in_buffer = (char *)audio_malloc(delay_size);
        AUDIO_MEM_CHECK(TAG, in_buffer, return ESP_FAIL);
#if SOC_I2S_SUPPORTS_ADC_DAC
        i2s_stream_t *i2s = (i2s_stream_t *)audio_element_getdata(i2s_stream);
        if ((i2s->config.i2s_config.mode & I2S_MODE_DAC_BUILT_IN) != 0) {
            memset(in_buffer, 0x80, delay_size);
        } else
#endif
        {
            memset(in_buffer, 0x00, delay_size);
        }
        ringbuf_handle_t input_rb = audio_element_get_input_ringbuf(i2s_stream);
        if (input_rb) {
            rb_write(input_rb, in_buffer, delay_size, 0);
        }
        audio_free(in_buffer);
    } else if (delay_ms > 0) {
        uint32_t drop_size = delay_ms * ((uint32_t)(info.sample_rates * info.channels * info.bits / 8) / 1000);
        in_buffer = (char *)audio_malloc(drop_size);
        AUDIO_MEM_CHECK(TAG, in_buffer, return ESP_FAIL);
        uint32_t r_size = audio_element_input(i2s_stream, in_buffer, drop_size);
        audio_free(in_buffer);

        if (r_size > 0) {
            audio_element_update_byte_pos(i2s_stream, r_size);
        } else {
            ESP_LOGW(TAG, "Can't get enough data to drop.");
            return ESP_FAIL;
        }
    }

    return ESP_OK;
}


/* -------------------------------------- Static Functions -------------------------------------------- */

#define I2S_CHANNEL_STD_INIT(i2s_handle, mode) do {                   \
    if (mode == AUDIO_STREAM_WRITER) {                                \
        ret = i2s_new_channel(&i2s_cfg->chan_cfg, &i2s_handle, NULL); \
    } else {                                                          \
        ret = i2s_new_channel(&i2s_cfg->chan_cfg, NULL, &i2s_handle); \
    }                                                                 \
    ret |= i2s_channel_init_std_mode(i2s_handle, &i2s_cfg->transmit_cfg.std_mode.std_config);\
    ret |= i2s_channel_enable(i2s_handle); \
} while (0)

#define I2S_CHANNEL_PDM_RX_INIT(i2s_handle) do { \
    ret = i2s_new_channel(&i2s_cfg->chan_cfg, NULL, &i2s->rx_handle); \
    ret |= i2s_channel_init_pdm_rx_mode(i2s->rx_handle, &i2s_cfg->transmit_cfg.pdm_mode.pdm_rx_config); \
    ret |= i2s_channel_enable(i2s->rx_handle);   \
} while (0)

#define I2S_CHANNEL_PDM_TX_INIT(i2s_handle) do { \
    ret = i2s_new_channel(&i2s_cfg->chan_cfg, &i2s->tx_handle, NULL); \
    ret |= i2s_channel_init_pdm_tx_mode(i2s->tx_handle, &i2s_cfg->transmit_cfg.pdm_mode.pdm_tx_config); \
    ret |= i2s_channel_enable(i2s->tx_handle);   \
} while (0)

#define I2S_CHANNEL_TDM_INIT(i2s_handle, mode) do {                    \
    if (mode == AUDIO_STREAM_WRITER) {                                 \
        ret = i2s_new_channel(&i2s_cfg->chan_cfg, &i2s_handle, NULL);  \
    } else {                                                           \
        ret = i2s_new_channel(&i2s_cfg->chan_cfg, NULL, &i2s_handle);  \
    }                                                                  \
    ret |= i2s_channel_init_tdm_mode(i2s_handle, &i2s_cfg->transmit_cfg.tdm_mode.tdm_config);\
    ret |= i2s_channel_enable(i2s_handle); \
} while (0)

static int i2s_driver_startup(audio_element_handle_t self, i2s_stream_cfg_t *i2s_cfg, i2s_stream_t *i2s)
{
    esp_err_t ret =ESP_OK;
#if (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0))
    i2s_cfg->chan_cfg.auto_clear = true;

        i2s_comm_mode_t i2s_mode = i2s_cfg->transmit_mode;
        board_i2s_pin_t board_i2s_pin = {0};
        switch(i2s_mode) {
            case I2S_COMM_MODE_STD:
                get_i2s_pins(i2s_cfg->chan_cfg.id, &board_i2s_pin);
                memcpy(&i2s_cfg->transmit_cfg.std_mode.std_config.gpio_cfg, &board_i2s_pin, sizeof(board_i2s_pin_t));
                if (i2s_cfg->type == AUDIO_STREAM_READER) {
                    I2S_CHANNEL_STD_INIT(i2s->rx_handle, AUDIO_STREAM_READER);
                } else if (i2s_cfg->type == AUDIO_STREAM_WRITER) {
                    I2S_CHANNEL_STD_INIT(i2s->tx_handle, AUDIO_STREAM_WRITER);
                }
                audio_element_set_music_info(self, i2s_cfg->transmit_cfg.std_mode.std_config.clk_cfg.sample_rate_hz ,
                                                i2s_cfg->transmit_cfg.std_mode.std_config.slot_cfg.slot_mode,
                                                i2s_cfg->transmit_cfg.std_mode.std_config.slot_cfg.slot_bit_width);
                break;
#if SOC_I2S_SUPPORTS_PDM
            case I2S_COMM_MODE_PDM:
                get_i2s_pins(i2s_cfg->chan_cfg.id, &board_i2s_pin);
                if (i2s_cfg->type == AUDIO_STREAM_READER) { 
                    i2s_cfg->transmit_cfg.pdm_mode.pdm_rx_config.gpio_cfg.clk = board_i2s_pin.bck_io_num;
                    i2s_cfg->transmit_cfg.pdm_mode.pdm_rx_config.gpio_cfg.din = board_i2s_pin.data_in_num;
                    I2S_CHANNEL_PDM_RX_INIT(i2s->rx_handle);
                    audio_element_set_music_info(self, i2s_cfg->transmit_cfg.pdm_mode.pdm_rx_config.clk_cfg.sample_rate_hz ,
                                                i2s_cfg->transmit_cfg.pdm_mode.pdm_rx_config.slot_cfg.slot_mode,
                                                i2s_cfg->transmit_cfg.pdm_mode.pdm_rx_config.slot_cfg.slot_bit_width);
                } else {
                    i2s_cfg->transmit_cfg.pdm_mode.pdm_tx_config.gpio_cfg.clk = board_i2s_pin.bck_io_num;
                    i2s_cfg->transmit_cfg.pdm_mode.pdm_tx_config.gpio_cfg.dout = board_i2s_pin.data_out_num;
                    I2S_CHANNEL_PDM_TX_INIT(i2s->tx_handle);
                    audio_element_set_music_info(self, i2s_cfg->transmit_cfg.pdm_mode.pdm_tx_config.clk_cfg.sample_rate_hz ,
                                                i2s_cfg->transmit_cfg.pdm_mode.pdm_tx_config.slot_cfg.slot_mode,
                                                i2s_cfg->transmit_cfg.pdm_mode.pdm_tx_config.slot_cfg.slot_bit_width);
                }

                break;
#endif // SOC_I2S_SUPPORTS_PDM

#if SOC_I2S_SUPPORTS_TDM
            case I2S_COMM_MODE_TDM:
                get_i2s_pins(i2s_cfg->chan_cfg.id, &board_i2s_pin);
                memcpy(&i2s_cfg->transmit_cfg.tdm_mode.tdm_config.gpio_cfg, &board_i2s_pin, sizeof(board_i2s_pin_t));
                if (i2s_cfg->type == AUDIO_STREAM_READER) {              
                    I2S_CHANNEL_TDM_INIT(i2s->rx_handle, AUDIO_STREAM_READER);
                } else if (i2s_cfg->type == AUDIO_STREAM_WRITER) {
                    I2S_CHANNEL_TDM_INIT(i2s->tx_handle, AUDIO_STREAM_WRITER);
                }

                audio_element_set_music_info(self, i2s_cfg->transmit_cfg.tdm_mode.tdm_config.clk_cfg.sample_rate_hz,
                                                i2s_cfg->transmit_cfg.tdm_mode.tdm_config.slot_cfg.slot_mode,
                                                i2s_cfg->transmit_cfg.tdm_mode.tdm_config.slot_cfg.slot_bit_width);
                break;
#endif // SOC_I2S_SUPPORTS_TDM

            default :
                ESP_LOGE(TAG, "Invalid i2s transmit mode");
                ret = ESP_FAIL;
                break;
        }
#else
    ret = i2s_driver_install(i2s->config.i2s_port, &i2s->config.i2s_config, 0, NULL);
    if (ret == ESP_ERR_INVALID_STATE) {
        ret = ESP_OK;
    }

    audio_element_set_music_info(self, i2s->config.i2s_config.sample_rate,
                                 i2s->config.i2s_config.channel_format < I2S_CHANNEL_FMT_ONLY_RIGHT ? 2 : 1,
                                 i2s->config.i2s_config.bits_per_sample);

#if SOC_I2S_SUPPORTS_ADC_DAC
    if ((config->i2s_config.mode & I2S_MODE_DAC_BUILT_IN) != 0) {
        i2s_set_dac_mode(I2S_DAC_CHANNEL_BOTH_EN);
    } else
#endif
    {
        board_i2s_pin_t board_i2s_pin = {0};
        i2s_pin_config_t i2s_pin_cfg;
        get_i2s_pins(i2s->config.i2s_port, &board_i2s_pin);
        i2s_pin_cfg.bck_io_num = board_i2s_pin.bck_io_num;
        i2s_pin_cfg.ws_io_num = board_i2s_pin.ws_io_num;
        i2s_pin_cfg.data_out_num = board_i2s_pin.data_out_num;
        i2s_pin_cfg.data_in_num = board_i2s_pin.data_in_num;
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(4, 4, 0)
        i2s_mclk_gpio_select(i2s->config.i2s_port, board_i2s_pin.mck_io_num);
#else
        i2s_pin_cfg.mck_io_num = board_i2s_pin.mck_io_num;
#endif
        i2s_set_pin(i2s->config.i2s_port, &i2s_pin_cfg);
    }
#endif
    return ret;
}

static int i2s_driver_cleanup(i2s_stream_t *i2s)
{
#if (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0))
    if (i2s->tx_handle) {
        i2s_channel_disable(i2s->tx_handle);
        i2s_del_channel(i2s->tx_handle);
    }
    if (i2s->rx_handle) {
        i2s_channel_disable(i2s->rx_handle);
        i2s_del_channel(i2s->rx_handle);
    }
#else
    i2s_driver_uninstall(i2s->config.i2s_port);
#endif
    return ESP_OK;
}

static int i2s_common_read(i2s_stream_t *i2s, char *buffer, size_t size,  TickType_t ticks_to_wait)
{
    size_t bytes_read = 0;
#if (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0))
    i2s_channel_read(i2s->rx_handle, buffer, size, &bytes_read, ticks_to_wait);
#else
    i2s_read(i2s->config.i2s_port, buffer, size, &bytes_read, ticks_to_wait);
#endif
    return bytes_read;
}

static int i2s_common_write(i2s_stream_t *i2s, char *buffer, size_t size,  TickType_t ticks_to_wait)
{
    size_t bytes_written = 0;
#if (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0))
    i2s_channel_write(i2s->tx_handle, buffer, size, &bytes_written, ticks_to_wait);
#else
    if (i2s->config.need_expand) {
            i2s_write_expand(i2s->config.i2s_port,
                    buffer,
                    size,
                    i2s->config.expand_src_bits,
                    i2s->target_bits,
                    &bytes_written,
                    ticks_to_wait);
    } else {
        i2s_write(i2s->config.i2s_port, buffer, size, &bytes_written, ticks_to_wait);
    }
#endif
    return bytes_written;
}

static esp_err_t i2s_common_set_clk(i2s_stream_t *i2s, int rate, int bits, int ch)
{
    esp_err_t err = ESP_OK;

#if (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0))

    i2s_chan_handle_t trans_handle = i2s->type == AUDIO_STREAM_READER ? i2s->rx_handle : i2s->tx_handle;
    if (i2s->config.transmit_mode == I2S_COMM_MODE_STD) {
        i2s_std_slot_config_t slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(bits, ch);
        i2s_std_clk_config_t clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(rate);
        if (bits == 24) {
            clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_384;
        }
        i2s_channel_disable(trans_handle);
        err |= i2s_channel_reconfig_std_slot(trans_handle, &slot_cfg);
        err = i2s_channel_reconfig_std_clock(trans_handle, &clk_cfg);
        i2s_channel_enable(trans_handle);
    } else if(i2s->config.transmit_mode == I2S_COMM_MODE_PDM) {
        if (i2s->type == AUDIO_STREAM_WRITER) {
            i2s_pdm_tx_slot_config_t slot_cfg = I2S_PDM_TX_SLOT_DEFAULT_CONFIG(bits, ch);
            i2s_pdm_tx_clk_config_t clk_cfg = I2S_PDM_TX_CLK_DEFAULT_CONFIG(rate);
            i2s_channel_disable(trans_handle);
            err |= i2s_channel_reconfig_pdm_tx_slot(trans_handle, &slot_cfg);
            err |= i2s_channel_reconfig_pdm_tx_clock(trans_handle, &clk_cfg);
            i2s_channel_enable(trans_handle);
        }
    }
#else
    i2s_channel_t channel;
    if (ch == 1) {
        channel = I2S_CHANNEL_MONO;
    } else if (ch == 2) {
        channel = I2S_CHANNEL_STEREO;
    } else {
        return ESP_FAIL;
    }
    err = i2s_set_clk(i2s->config.i2s_port, rate, bits, channel);
#endif
    return err;
}
