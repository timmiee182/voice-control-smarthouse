#include "audio_capture.h"
#include "mfcc_processor.h"
#include "kws_inference.h"
#include "device_control.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>
#include <math.h>
#include "esp_timer.h"

static const char* TAG = "KWS";

#define N_MFCC   40
#define N_FRAMES 49
#define VAD_THRESHOLD  0.01f
#define COOLDOWN_MS     1500
// Buffer lưu MFCC raw [n_frames, n_mfcc] = [49, 40]
static float s_mfcc_raw[N_FRAMES * N_MFCC];

// Buffer cuối cùng gửi vào model [n_mfcc, n_frames, 3] = [40, 49, 3]
static float s_model_input[N_MFCC * N_FRAMES * 3];

// Tính delta từ MFCC matrix
// input/output shape: [n_mfcc, n_frames]
static void compute_delta(const float* in, float* out, int n_mfcc, int n_frames) {
    for (int m = 0; m < n_mfcc; m++) {
        for (int t = 0; t < n_frames; t++) {
            // Dùng công thức delta đơn giản: trung bình có trọng số
            float num = 0.0f, den = 0.0f;
            for (int k = 1; k <= 2; k++) {
                int t_fwd = (t + k < n_frames) ? t + k : n_frames - 1;
                int t_bwd = (t - k >= 0)        ? t - k : 0;
                num += (float)k * (in[m * n_frames + t_fwd] - in[m * n_frames + t_bwd]);
                den += 2.0f * k * k;
            }
            out[m * n_frames + t] = num / (den + 1e-8f);
        }
    }
}

static float compute_rms(const int16_t* pcm, size_t n) {
    float sum = 0.0f;
    for (size_t i = 0; i < n; i++) {
        float s = (float)pcm[i] / 32768.0f;
        sum += s * s;
    }
    return sqrtf(sum / n);
}

static int64_t s_last_trigger_ms = 0;

static void on_audio_frame(const int16_t* pcm, size_t n) {

    float rms = compute_rms(pcm, n);
    if (rms < VAD_THRESHOLD) {
        // Uncomment dòng dưới để debug xem RMS thực tế là bao nhiêu
        // ESP_LOGD(TAG, "Silence RMS=%.4f < %.4f, skip", rms, VAD_THRESHOLD);
        return;
    }
        // Cooldown check
    int64_t now_ms = esp_timer_get_time() / 1000;
    if ((now_ms - s_last_trigger_ms) < COOLDOWN_MS) {
        ESP_LOGD(TAG, "Cooldown, skip (%.0f ms remaining)",
                 (float)(COOLDOWN_MS - (now_ms - s_last_trigger_ms)));
        return;
    }
    ESP_LOGI(TAG, "Voice detected RMS=%.4f", rms);
    // ── Bước 1: Tính MFCC raw → s_mfcc_raw[49, 40] ──────────────
    compute_mfcc(pcm, s_mfcc_raw);
    // s_mfcc_raw layout: [frame][coeff] = [49][40]

    // ── Bước 2: Chuyển sang [n_mfcc, n_frames] = [40, 49] ────────
    // Để tính delta đúng cách (theo chiều thời gian)
    static float mfcc_T[N_MFCC * N_FRAMES];   // [40, 49]
    for (int m = 0; m < N_MFCC; m++)
        for (int t = 0; t < N_FRAMES; t++)
            mfcc_T[m * N_FRAMES + t] = s_mfcc_raw[t * N_MFCC + m];

    // ── Bước 3: Tính delta và delta2 ─────────────────────────────
    static float delta[N_MFCC * N_FRAMES];
    static float delta2[N_MFCC * N_FRAMES];
    compute_delta(mfcc_T, delta,  N_MFCC, N_FRAMES);
    compute_delta(delta,  delta2, N_MFCC, N_FRAMES);

    // ── Bước 4: Pack vào model input [40, 49, 3] ─────────────────
    // Layout: index = m * N_FRAMES * 3 + t * 3 + channel
    for (int m = 0; m < N_MFCC; m++) {
        for (int t = 0; t < N_FRAMES; t++) {
            int idx = m * N_FRAMES * 3 + t * 3;
            s_model_input[idx + 0] = mfcc_T [m * N_FRAMES + t];  // ch0: MFCC
            s_model_input[idx + 1] = delta  [m * N_FRAMES + t];  // ch1: delta
            s_model_input[idx + 2] = delta2 [m * N_FRAMES + t];  // ch2: delta2
        }
    }

    // ── Bước 5: Inference ─────────────────────────────────────────
    float confidence = 0.0f;
    int class_idx = kws_run(s_model_input, &confidence);

    // Log tất cả kết quả (kể cả unknown) để debug
    ESP_LOGI(TAG, ">> %s (%.2f) RMS=%.4f", kws_class_name(class_idx), confidence, rms);

    // ── Bước 6: Điều khiển thiết bị ──────────────────────────────
    if (class_idx != IDX_UNKNOWN) {\
        s_last_trigger_ms = now_ms;
        device_control_handle(class_idx);
    }
}

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "KWS System starting...");

    mfcc_processor_init();

    if (!kws_inference_init()) {
        ESP_LOGE(TAG, "TFLite init FAILED");
        return;
    }

    device_control_init();
    audio_capture_init(on_audio_frame);
    audio_capture_start();

    ESP_LOGI(TAG, "Listening...");
    while (true) vTaskDelay(pdMS_TO_TICKS(1000));
}