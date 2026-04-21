#pragma once
#include <stdint.h>

#define SAMPLE_RATE    16000
#define FRAME_SAMPLES  16000

void mfcc_processor_init(void);
void compute_mfcc(const int16_t* pcm, float* out_features);