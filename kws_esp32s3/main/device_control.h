// device_control.h
#pragma once

// Index class khớp với CLASSES trong Python
#define IDX_BAT_DEN  0
#define IDX_TAT_DEN  1
#define IDX_BAT_QUAT 2
#define IDX_TAT_QUAT 3
#define IDX_BAT_HET  4
#define IDX_TAT_HET  5
#define IDX_UNKNOWN  6

void device_control_init(void);
void device_control_handle(int class_idx);