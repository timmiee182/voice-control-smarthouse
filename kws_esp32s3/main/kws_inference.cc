// kws_inference.cc
#include "kws_inference.h"
#include "esp_system.h"
#include "model/kws_model_data.h"
#include "esp_heap_caps.h"
// ── TFLite Micro (API mới — không dùng micro_error_reporter) ─────
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/micro_log.h"          // thay cho error_reporter
#include "tensorflow/lite/schema/schema_generated.h"

#include <math.h>
#include <string.h>
#include "esp_log.h"

static const char* TAG = "KWS_INFER";

// ── Thông số quantization — lấy từ quant_info.json của bạn ───────
#define INPUT_SCALE        0.10073702037334442f
#define INPUT_ZERO_POINT   79
#define OUTPUT_SCALE       0.00390625f
#define OUTPUT_ZERO_POINT  (-128)
#define CONFIDENCE_THRESH  0.80f
#define NUM_CLASSES        7
#define IDX_UNKNOWN        6

static const char* CLASS_NAMES[] = {
    "bat_den", "tat_den", "bat_quat",
    "tat_quat", "bat_het", "tat_het", "unknown"
};

// ── TFLite Micro objects ──────────────────────────────────────────
static const tflite::Model*       s_model      = nullptr;
static tflite::MicroInterpreter*  s_interpreter = nullptr;
static TfLiteTensor*              s_input       = nullptr;
static TfLiteTensor*              s_output      = nullptr;

// Tensor arena — tăng lên nếu AllocateTensors() thất bại
constexpr int TENSOR_ARENA_SIZE = 72 * 1024;
static uint8_t* s_tensor_arena = nullptr;

// Chỉ đăng ký đúng ops model dùng — giảm Flash
using Resolver = tflite::MicroMutableOpResolver<8>;
static Resolver s_resolver;

bool kws_inference_init(void) {
    // Thử theo thứ tự ưu tiên
    const uint32_t caps[] = {
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT,
        MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT,
        MALLOC_CAP_DEFAULT
    };
    
    for (int i = 0; i < 3 && !s_tensor_arena; i++) {
        s_tensor_arena = (uint8_t*)heap_caps_malloc(
            TENSOR_ARENA_SIZE, caps[i]
        );
        if (s_tensor_arena) {
            ESP_LOGI(TAG, "Arena allocated with caps[%d]: %d KB at %p",
                     i, TENSOR_ARENA_SIZE/1024, s_tensor_arena);
        }
    }

    if (!s_tensor_arena) {
        // Log thông tin heap để debug
        ESP_LOGE(TAG, "Alloc thất bại! Free heap: %lu, Free SPIRAM: %lu",
                 (unsigned long)esp_get_free_heap_size(),
                 (unsigned long)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
        return false;
    }
    ESP_LOGI(TAG, "Arena: %d KB tại 0x%p", TENSOR_ARENA_SIZE/1024, s_tensor_arena);
    // Đăng ký ops
    s_resolver.AddConv2D();
    s_resolver.AddDepthwiseConv2D();
    s_resolver.AddRelu();
    s_resolver.AddRelu6();
    s_resolver.AddMean();               // GlobalAveragePooling2D
    s_resolver.AddFullyConnected();
    s_resolver.AddSoftmax();
    s_resolver.AddReshape();

    // Load model từ C array
    s_model = tflite::GetModel(kws_model_data);
    if (s_model->version() != TFLITE_SCHEMA_VERSION) {
        ESP_LOGE(TAG, "Model schema version không khớp: %d != %d",
                 s_model->version(), TFLITE_SCHEMA_VERSION);
        return false;
    }

    // Tạo interpreter — API mới không cần error_reporter
    static tflite::MicroInterpreter interpreter(
        s_model, s_resolver,
        s_tensor_arena, TENSOR_ARENA_SIZE
    );
    s_interpreter = &interpreter;

    // Cấp phát bộ nhớ cho tensors
    TfLiteStatus status = s_interpreter->AllocateTensors();
    if (status != kTfLiteOk) {
        ESP_LOGE(TAG, "AllocateTensors thất bại — thử tăng TENSOR_ARENA_SIZE");
        return false;
    }

    s_input  = s_interpreter->input(0);
    s_output = s_interpreter->output(0);

    ESP_LOGI(TAG, "TFLite Micro OK");
    ESP_LOGI(TAG, "Arena used: %d / %d bytes",
             (int)s_interpreter->arena_used_bytes(), TENSOR_ARENA_SIZE);
    ESP_LOGI(TAG, "Input  shape: [%d, %d, %d, %d]",
             s_input->dims->data[0], s_input->dims->data[1],
             s_input->dims->data[2], s_input->dims->data[3]);

    return true;
}

int kws_run(const float* mfcc_features, float* out_confidence) {
    if (!s_interpreter) return IDX_UNKNOWN;

    // Quantize input: float → int8
    int n = s_input->bytes;
    for (int i = 0; i < n; i++) {
        int32_t q = (int32_t)roundf(mfcc_features[i] / INPUT_SCALE)
                    + INPUT_ZERO_POINT;
        if (q < -128) q = -128;
        if (q >  127) q =  127;
        s_input->data.int8[i] = (int8_t)q;
    }

    // Inference
    if (s_interpreter->Invoke() != kTfLiteOk) {
        ESP_LOGE(TAG, "Invoke thất bại");
        return IDX_UNKNOWN;
    }
    // Log tất cả scores
    float scores[NUM_CLASSES];
    for (int i = 0; i < NUM_CLASSES; i++) {
        scores[i] = ((float)s_output->data.int8[i] - OUTPUT_ZERO_POINT)
                    * OUTPUT_SCALE;
    }
    ESP_LOGI(TAG, "Scores: bat_den=%.2f tat_den=%.2f bat_quat=%.2f "
                  "tat_quat=%.2f bat_het=%.2f tat_het=%.2f unk=%.2f",
             scores[0], scores[1], scores[2],
             scores[3], scores[4], scores[5], scores[6]);
    // Dequantize output + tìm class tốt nhất
    int   best_idx  = IDX_UNKNOWN;
    float best_prob = 0.0f;

    for (int i = 0; i < NUM_CLASSES; i++) {
        float prob = ((float)s_output->data.int8[i] - OUTPUT_ZERO_POINT)
                     * OUTPUT_SCALE;
        if (prob > best_prob) {
            best_prob = prob;
            best_idx  = i;
        }
    }

    if (out_confidence) *out_confidence = best_prob;

    // Chỉ trả kết quả nếu vượt ngưỡng
    return (best_prob >= CONFIDENCE_THRESH) ? best_idx : IDX_UNKNOWN;
}

const char* kws_class_name(int idx) {
    if (idx < 0 || idx >= NUM_CLASSES) return "unknown";
    return CLASS_NAMES[idx];
}