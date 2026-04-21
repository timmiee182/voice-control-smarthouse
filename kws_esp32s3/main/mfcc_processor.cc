#include "mfcc_processor.h"
#include "dsps_fft2r.h"
#include "dsps_wind_hann.h"
#include "esp_log.h"
#include <math.h>
#include <string.h>

static const char* TAG = "MFCC";

#define N_MFCC      40
#define HOP_LENGTH  320
#define N_FFT       512
#define N_FRAMES    49
#define N_FILTERS   40

// ── Buffers tĩnh — khai báo ngoài hàm để đảm bảo alignment ──────
static __attribute__((aligned(16))) float s_fft_buf[N_FFT * 2];
static __attribute__((aligned(16))) float s_hann[N_FFT];
static float s_audio_f[FRAME_SAMPLES];
static float s_frame[N_FFT];
static float s_power[N_FFT / 2 + 1];
static float s_mel_energy[N_FILTERS];
static float s_mfcc_frame[N_MFCC];
static float s_mel_fb[N_FILTERS][N_FFT / 2 + 1];
static float s_dct[N_MFCC][N_FILTERS];

static void init_mel_filterbank(void) {
    auto hz2mel = [](float f){ return 2595.0f * log10f(1.0f + f / 700.0f); };
    auto mel2hz = [](float m){ return 700.0f * (powf(10.0f, m / 2595.0f) - 1.0f); };

    float mel_min = hz2mel(0.0f);
    float mel_max = hz2mel((float)SAMPLE_RATE / 2.0f);

    float mel_points[N_FILTERS + 2];
    for (int i = 0; i < N_FILTERS + 2; i++)
        mel_points[i] = mel2hz(mel_min + (mel_max - mel_min) * i / (N_FILTERS + 1));

    int bins[N_FILTERS + 2];
    for (int i = 0; i < N_FILTERS + 2; i++)
        bins[i] = (int)floorf((N_FFT + 1) * mel_points[i] / SAMPLE_RATE);

    memset(s_mel_fb, 0, sizeof(s_mel_fb));
    for (int m = 1; m <= N_FILTERS; m++) {
        for (int k = bins[m-1]; k < bins[m]; k++)
            s_mel_fb[m-1][k] = (float)(k - bins[m-1]) / (bins[m] - bins[m-1]);
        for (int k = bins[m]; k < bins[m+1]; k++)
            s_mel_fb[m-1][k] = (float)(bins[m+1] - k) / (bins[m+1] - bins[m]);
    }
}

static void init_dct(void) {
    for (int i = 0; i < N_MFCC; i++)
        for (int j = 0; j < N_FILTERS; j++)
            s_dct[i][j] = cosf(M_PI * i * (2*j + 1) / (2.0f * N_FILTERS));
}

void mfcc_processor_init(void) {
    // Init Hann window qua ESP-DSP
    dsps_wind_hann_f32(s_hann, N_FFT);

    // Init FFT tables — BẮT BUỘC gọi trước dsps_fft2r_fc32
    esp_err_t ret = dsps_fft2r_init_fc32(nullptr, N_FFT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "FFT init thất bại: %d", ret);
    }

    init_mel_filterbank();
    init_dct();
    ESP_LOGI(TAG, "MFCC processor OK");
}

static void pre_emphasis(const int16_t* in, size_t n) {
    s_audio_f[0] = (float)in[0] / 32768.0f;
    for (size_t i = 1; i < n; i++) {
        float cur  = (float)in[i]   / 32768.0f;
        float prev = (float)in[i-1] / 32768.0f;
        s_audio_f[i] = cur - 0.97f * prev;
    }
}

void compute_mfcc(const int16_t* pcm, float* out_features) {
    pre_emphasis(pcm, FRAME_SAMPLES);

    for (int t = 0; t < N_FRAMES; t++) {
        int start = t * HOP_LENGTH;

        // Lấy frame + apply Hann window
        for (int i = 0; i < N_FFT; i++) {
            int idx = start + i;
            float sample = (idx < FRAME_SAMPLES) ? s_audio_f[idx] : 0.0f;
            s_fft_buf[2*i]     = sample * s_hann[i];  // real
            s_fft_buf[2*i + 1] = 0.0f;                // imag
        }

        // FFT — dùng buffer đã aligned
        dsps_fft2r_fc32(s_fft_buf, N_FFT);
        dsps_bit_rev_fc32(s_fft_buf, N_FFT);

        // Power spectrum
        for (int i = 0; i <= N_FFT / 2; i++) {
            float re = s_fft_buf[2*i];
            float im = s_fft_buf[2*i + 1];
            s_power[i] = re*re + im*im;
        }

        // Mel filterbank
        for (int m = 0; m < N_FILTERS; m++) {
            float energy = 0.0f;
            for (int k = 0; k <= N_FFT / 2; k++)
                energy += s_mel_fb[m][k] * s_power[k];
            s_mel_energy[m] = logf(energy + 1e-8f);
        }

        // DCT → MFCC
        for (int i = 0; i < N_MFCC; i++) {
            float v = 0.0f;
            for (int j = 0; j < N_FILTERS; j++)
                v += s_dct[i][j] * s_mel_energy[j];
            s_mfcc_frame[i] = v;
        }

        // Layout output: [frame, mfcc] → index = t*N_MFCC + coeff
        memcpy(out_features + t * N_MFCC, s_mfcc_frame, N_MFCC * sizeof(float));
    }

    // Normalize toàn bộ feature map
    float mean = 0.0f, std = 0.0f;
    int total = N_MFCC * N_FRAMES;
    for (int i = 0; i < total; i++) mean += out_features[i];
    mean /= total;
    for (int i = 0; i < total; i++) {
        float d = out_features[i] - mean;
        std += d * d;
    }
    std = sqrtf(std / total + 1e-6f);
    for (int i = 0; i < total; i++)
        out_features[i] = (out_features[i] - mean) / std;
}