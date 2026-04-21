// kws_inference.h
#pragma once

bool        kws_inference_init(void);
int         kws_run(const float* mfcc_features, float* out_confidence);
const char* kws_class_name(int idx);
