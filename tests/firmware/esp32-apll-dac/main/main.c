/* Real ESP-IDF ESP32 I2C master and I2S DMA firmware; no SDK interception.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "driver/i2c_master.h"
#include "driver/i2s_std.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/regi2c_ctrl.h"
#include "soc/regi2c_apll.h"

#define DAC_ADDRESS 0x4c
#ifndef APLL_DAC_RATE
#define APLL_DAC_RATE 16000
#endif
#ifndef APLL_DAC_I2S_PORT
#define APLL_DAC_I2S_PORT 0
#endif
#define RATE APLL_DAC_RATE
#define FRAMES (RATE * 5)
#define BLOCK_FRAMES 256
#define DMA_DESC_NUM 4
#define I2C_TIMEOUT_MS 1000
#if RATE == 16000
#define DAC_RATE_ID 1
#elif RATE == 32000
#define DAC_RATE_ID 2
#else
#error "This fixture contains validated 16 kHz and 32 kHz profiles"
#endif
#define STATUS_CLOCK_ERROR 0x01
#define STATUS_CONFIG_ERROR 0x02
#define STATUS_RECORDING 0x04

/* 32767 * sqrt(0.5) * sin(2*pi*n/16), rounded to the nearest integer.
 * A 16-point cycle at 16 kHz is exactly 1 kHz, with no phase drift.
 */
#if RATE != 32000
static const int16_t cycle[16] = {
    0, 8867, 16384, 21407, 23170, 21407, 16384, 8867,
    0, -8867, -16384, -21407, -23170, -21407, -16384, -8867
};
#endif
#if RATE == 32000
static const int16_t cycle_32k[32] = {
    0, 4520, 8867, 12846, 16384, 19308, 21407, 22725,
    23170, 22725, 21407, 19308, 16384, 12846, 8867, 4520,
    0, -4520, -8867, -12846, -16384, -19308, -21407, -22725,
    -23170, -22725, -21407, -19308, -16384, -12846, -8867, -4520,
};
#endif

static void dac_write(i2c_master_dev_handle_t dac, uint8_t reg, uint8_t value)
{
    uint8_t bytes[2] = {reg, value};
    ESP_ERROR_CHECK(i2c_master_transmit(dac, bytes, sizeof(bytes), I2C_TIMEOUT_MS));
    printf("DAC_WRITE reg=%02x value=%02x\n", reg, value);
}

static uint8_t dac_status(i2c_master_dev_handle_t dac, bool recording)
{
    uint8_t reg = 0x04, reply[5] = {0xff, 0xff, 0xff, 0xff, 0xff};
    /* Read status and the latched frame counter in one legal burst.
     * The pinned IDF v6.1 one-byte branch emits READ byte_num=0 on ESP32
     * (s_i2c_read_command), outside TRM 21.3.4's specified 1..255 range. */
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dac, &reg, 1, reply,
                                              sizeof(reply), I2C_TIMEOUT_MS));
    uint8_t status = reply[0];
    uint32_t frames = (uint32_t)reply[1] | ((uint32_t)reply[2] << 8) |
                      ((uint32_t)reply[3] << 16) | ((uint32_t)reply[4] << 24);
    printf("DAC_STATUS phase=%s value=%02x\n", recording ? "recording" : "idle", status);
    printf("DAC_COUNTER phase=%s frames=%lu\n", recording ? "recording" : "idle",
           (unsigned long)frames);
    if (status & (STATUS_CLOCK_ERROR | STATUS_CONFIG_ERROR) ||
        !!(status & STATUS_RECORDING) != recording) {
        printf("VIRTUAL_DAC_FAIL status=%02x expected_recording=%d\n", status, recording);
        abort();
    }
    return status;
}

void app_main(void)
{
    printf("VIRTUAL_DAC_BEGIN frames=%d rate=%d channels=2 bits=16 i2s=%d\n",
           FRAMES, RATE, APLL_DAC_I2S_PORT);
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t dac;
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0, .sda_io_num = 21, .scl_io_num = 22,
        .clk_source = I2C_CLK_SRC_DEFAULT, .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = DAC_ADDRESS, .scl_speed_hz = 100000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &dev_cfg, &dac));
    dac_status(dac, false);
    dac_write(dac, 0x00, 2);     /* stereo */
    dac_write(dac, 0x01, 16);    /* signed 16-bit PCM */
    dac_write(dac, 0x02, DAC_RATE_ID);
    dac_status(dac, false);

    i2s_chan_handle_t tx;
#if APLL_DAC_I2S_PORT == 0
    const int i2s_port = I2S_NUM_0;
    const int bclk_gpio = 18, ws_gpio = 19, dout_gpio = 23;
#elif APLL_DAC_I2S_PORT == 1
    const int i2s_port = I2S_NUM_1;
    const int bclk_gpio = 5, ws_gpio = 25, dout_gpio = 26;
#else
#error "Only the two original ESP32 I2S controllers are modeled"
#endif
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(i2s_port, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = DMA_DESC_NUM;
    chan_cfg.dma_frame_num = BLOCK_FRAMES;
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx, NULL));
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                        I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED, .bclk = bclk_gpio, .ws = ws_gpio,
            .dout = dout_gpio,
            .din = I2S_GPIO_UNUSED,
        },
    };
    /* Exercise the unmodified IDF APLL frequency search, ROM analog-I2C
     * routines and hardware calibration loop through the real I2S API. */
    std_cfg.clk_cfg.clk_src = I2S_CLK_SRC_APLL;
    /* MCLK is intentionally not routed: the DAC measures the frame rate
     * from WS edges and the BCLK count per frame. Routing MCLK would add
     * ~40M pin events to a 5-second run with no verification content
     * beyond what WS rate plus BCLK-per-frame already prove. */
    std_cfg.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx, &std_cfg));
    unsigned cal_end = REGI2C_READ_MASK(I2C_APLL, I2C_APLL_OR_CAL_END);
    unsigned odiv = REGI2C_READ_MASK(I2C_APLL, I2C_APLL_OR_OUTPUT_DIV);
    unsigned sdm2 = REGI2C_READ_MASK(I2C_APLL, I2C_APLL_DSDM2);
    unsigned sdm1 = REGI2C_READ_MASK(I2C_APLL, I2C_APLL_DSDM1);
    unsigned sdm0 = REGI2C_READ_MASK(I2C_APLL, I2C_APLL_DSDM0);
    printf("APLL_CHECK cal_end=%u odiv=%u sdm=%u:%u:%u\n",
           cal_end, odiv, sdm2, sdm1, sdm0);
    if (!cal_end) {
        printf("VIRTUAL_DAC_FAIL APLL calibration did not complete\n");
        abort();
    }
    ESP_ERROR_CHECK(i2s_channel_enable(tx));
    ESP_ERROR_CHECK(i2s_channel_disable(tx));
    ESP_ERROR_CHECK(i2s_channel_enable(tx));
    dac_write(dac, 0x03, 1);
    dac_status(dac, true);
    int16_t samples[BLOCK_FRAMES * 2];
    for (unsigned frame = 0; frame < FRAMES; frame += BLOCK_FRAMES) {
        unsigned count = FRAMES - frame;
        if (count > BLOCK_FRAMES) count = BLOCK_FRAMES;
        for (unsigned i = 0; i < count; ++i) {
#if RATE == 32000
            int16_t value = cycle_32k[(frame + i) % 32];
#else
            int16_t value = cycle[(frame + i) % 16];
#endif
            samples[2 * i] = value;
            samples[2 * i + 1] = value;
        }
        size_t written = 0;
        size_t bytes = count * 2 * sizeof(int16_t);
        ESP_ERROR_CHECK(i2s_channel_write(tx, samples, bytes, &written, 1000));
        if (written != bytes) {
            printf("VIRTUAL_DAC_FAIL dma frame=%u wrote=%u expected=%u\n",
                   frame, (unsigned)written, (unsigned)bytes);
            abort();
        }
        if (frame <= FRAMES / 2 && frame + count > FRAMES / 2) dac_status(dac, true);
    }
    /* Replace the final DMA descriptor contents with silence so any
     * descriptor replays during the bounded drain interval are observable
     * as silence after the exact five-second tone. */
    memset(samples, 0, sizeof(samples));
    for (unsigned desc = 0; desc < DMA_DESC_NUM; ++desc) {
        size_t drain_written = 0;
        ESP_ERROR_CHECK(i2s_channel_write(tx, samples, sizeof(samples),
                                         &drain_written, 1000));
        if (drain_written != sizeof(samples)) {
            printf("VIRTUAL_DAC_FAIL drain wrote=%u expected=%u\n",
                   (unsigned)drain_written, (unsigned)sizeof(samples));
            abort();
        }
    }
    /* The ring holds four 256-frame descriptors (64 ms at 16 kHz). */
    vTaskDelay(pdMS_TO_TICKS(100));
    dac_write(dac, 0x03, 0);
    dac_status(dac, false);
    ESP_ERROR_CHECK(i2s_channel_disable(tx));
    ESP_ERROR_CHECK(i2s_del_channel(tx));
    ESP_ERROR_CHECK(i2c_master_bus_rm_device(dac));
    ESP_ERROR_CHECK(i2c_del_master_bus(bus));
    printf("VIRTUAL_DAC_PASS frames=%d channels=2 bits=16 rate=%d i2s=%d\n",
           FRAMES, RATE, APLL_DAC_I2S_PORT);
}
