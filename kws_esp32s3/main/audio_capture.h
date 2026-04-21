// audio_capture.h
#pragma once
#include <stdint.h>
#include <stddef.h>

#define SAMPLE_RATE     16000
#define FRAME_MS        1000          // cửa sổ 1 giây
#define FRAME_SAMPLES   (SAMPLE_RATE) // 16000 mẫu
#define SLIDE_MS        500           // trượt 500ms → overlap 50%
#define SLIDE_SAMPLES   (SAMPLE_RATE / 2)

// Gọi hàm này khi có 1 frame mới — implement trong main.cc
typedef void (*frame_callback_t)(const int16_t* pcm, size_t n_samples);

void audio_capture_init(frame_callback_t cb);
void audio_capture_start(void);