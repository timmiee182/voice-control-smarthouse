// audio_capture.cc
#include "audio_capture.h"
#include "driver/i2s_std.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

// ── GPIO theo sơ đồ của bạn ──────────────────────────────────────
#define PIN_SD      4    // mic_sd  → GPIO4
#define PIN_WS      5    // mic_ws  → GPIO5
#define PIN_SCK     6    // mic_sck → GPIO6

#define I2S_PORT    I2S_NUM_0

static frame_callback_t s_frame_cb = nullptr;
static i2s_chan_handle_t s_rx_chan;

// Circular buffer — 1.5s để sliding window 50%
static int16_t s_ring[FRAME_SAMPLES + SLIDE_SAMPLES];
static size_t  s_write_pos = 0;
static size_t  s_total     = 0;

static void i2s_reader_task(void* arg) {
    int32_t raw[256];
    size_t  bytes_read;

    while (true) {
        i2s_channel_read(s_rx_chan, raw, sizeof(raw), &bytes_read, portMAX_DELAY);
        int n = bytes_read / sizeof(int32_t);

        for (int i = 0; i < n; i++) {
            // INMP441: data 18-bit, MSB tại bit 31 (left-justified)
            int16_t sample = (int16_t)(raw[i] >> 14);

            s_ring[s_write_pos] = sample;
            s_write_pos = (s_write_pos + 1) % (FRAME_SAMPLES + SLIDE_SAMPLES);
            s_total++;

            // Mỗi SLIDE_SAMPLES mẫu → callback 1 frame đầy đủ
            if (s_total >= FRAME_SAMPLES && (s_total % SLIDE_SAMPLES) == 0) {
                static int16_t frame[FRAME_SAMPLES];
                size_t start = s_write_pos; // điểm ngay sau write = oldest
                for (size_t j = 0; j < FRAME_SAMPLES; j++)
                    frame[j] = s_ring[(start + j) % (FRAME_SAMPLES + SLIDE_SAMPLES)];
                if (s_frame_cb) s_frame_cb(frame, FRAME_SAMPLES);
            }
        }
    }
}

void audio_capture_init(frame_callback_t cb) {
    s_frame_cb = cb;

    i2s_chan_config_t chan_cfg =
        I2S_CHANNEL_DEFAULT_CONFIG(I2S_PORT, I2S_ROLE_MASTER);
    i2s_new_channel(&chan_cfg, nullptr, &s_rx_chan);

    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(
                        I2S_DATA_BIT_WIDTH_32BIT,
                        I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = (gpio_num_t)PIN_SCK,
            .ws   = (gpio_num_t)PIN_WS,
            .dout = I2S_GPIO_UNUSED,
            .din  = (gpio_num_t)PIN_SD,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };

    i2s_channel_init_std_mode(s_rx_chan, &std_cfg);
}

void audio_capture_start(void) {
    i2s_channel_enable(s_rx_chan);
    xTaskCreatePinnedToCore(
        i2s_reader_task, "i2s_reader",
        4096, nullptr, 5, nullptr,
        1   // Core 1 — Core 0 để inference
    );
}