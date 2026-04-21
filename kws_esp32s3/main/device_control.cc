// device_control.cc
#include "device_control.h"
#include "driver/gpio.h"
#include "esp_log.h"

// Dòng này dùng đúng tên array
static const char* TAG = "DEVICE";

// ── GPIO theo sơ đồ ──────────────────────────────────────────────
// Relay — module thường ACTIVE LOW (IN1=LOW → relay bật)
// Kiểm tra module của bạn: nếu active HIGH thì đổi ON/OFF bên dưới
#define PIN_RELAY_DEN   41   // IN1 → GPIO38 → điều khiển đèn
#define PIN_RELAY_QUAT  42   // IN2 → GPIO39 → điều khiển quạt

// RGB LED — common cathode (HIGH = sáng) hoặc common anode (LOW = sáng)
// Sơ đồ không thấy điện trở hạn dòng — kiểm tra lại module RGB của bạn
#define PIN_LED_R  17   // R → GPIO17
#define PIN_LED_G  18   // G → GPIO18
#define PIN_LED_B   8   // B → GPIO8

// Relay thường ACTIVE LOW — đổi 2 define này nếu module của bạn khác
#define RELAY_ON   1
#define RELAY_OFF  0

// Trạng thái hiện tại
static bool s_den_on  = false;
static bool s_quat_on = false;

// ── LED status indicator ──────────────────────────────────────────
typedef enum { LED_IDLE, LED_DEN, LED_QUAT, LED_ALL, LED_ERROR } led_state_t;

static void set_led(led_state_t state) {
    // R  G  B
    switch (state) {
        case LED_IDLE:  // Xanh lam — đang lắng nghe
            gpio_set_level((gpio_num_t)PIN_LED_R, 0);
            gpio_set_level((gpio_num_t)PIN_LED_G, 0);
            gpio_set_level((gpio_num_t)PIN_LED_B, 1);
            break;
        case LED_DEN:   // Vàng — đèn đang bật
            gpio_set_level((gpio_num_t)PIN_LED_R, 1);
            gpio_set_level((gpio_num_t)PIN_LED_G, 1);
            gpio_set_level((gpio_num_t)PIN_LED_B, 0);
            break;
        case LED_QUAT:  // Xanh lá — quạt đang bật
            gpio_set_level((gpio_num_t)PIN_LED_R, 0);
            gpio_set_level((gpio_num_t)PIN_LED_G, 1);
            gpio_set_level((gpio_num_t)PIN_LED_B, 0);
            break;
        case LED_ALL:   // Trắng — bật hết
            gpio_set_level((gpio_num_t)PIN_LED_R, 1);
            gpio_set_level((gpio_num_t)PIN_LED_G, 1);
            gpio_set_level((gpio_num_t)PIN_LED_B, 1);
            break;
        case LED_ERROR: // Đỏ — lỗi
            gpio_set_level((gpio_num_t)PIN_LED_R, 1);
            gpio_set_level((gpio_num_t)PIN_LED_G, 0);
            gpio_set_level((gpio_num_t)PIN_LED_B, 0);
            break;
    }
}

static void update_led(void) {
    if (s_den_on && s_quat_on)       set_led(LED_ALL);
    else if (s_den_on)               set_led(LED_DEN);
    else if (s_quat_on)              set_led(LED_QUAT);
    else                             set_led(LED_IDLE);
}

// ── Init ──────────────────────────────────────────────────────────
void device_control_init(void) {
    // Relay
    gpio_config_t relay_cfg = {
        .pin_bit_mask = (1ULL << PIN_RELAY_DEN) | (1ULL << PIN_RELAY_QUAT),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&relay_cfg);

    // RGB LED
    gpio_config_t led_cfg = {
        .pin_bit_mask = (1ULL << PIN_LED_R) |
                        (1ULL << PIN_LED_G) |
                        (1ULL << PIN_LED_B),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&led_cfg);

    // Tắt hết khi khởi động
    gpio_set_level((gpio_num_t)PIN_RELAY_DEN,  RELAY_OFF);
    gpio_set_level((gpio_num_t)PIN_RELAY_QUAT, RELAY_OFF);
    update_led();

    ESP_LOGI(TAG, "GPIO init OK — Relay DEN=%d QUAT=%d | LED R=%d G=%d B=%d",
             PIN_RELAY_DEN, PIN_RELAY_QUAT, PIN_LED_R, PIN_LED_G, PIN_LED_B);
}

// ── Xử lý lệnh ───────────────────────────────────────────────────
void device_control_handle(int class_idx) {
    switch (class_idx) {
        case IDX_BAT_DEN:
            if (!s_den_on) {
                s_den_on = true;
                gpio_set_level((gpio_num_t)PIN_RELAY_DEN, RELAY_ON);
                ESP_LOGI(TAG, "ĐÈN → BẬT");
            }
            break;

        case IDX_TAT_DEN:
            if (s_den_on) {
                s_den_on = false;
                gpio_set_level((gpio_num_t)PIN_RELAY_DEN, RELAY_OFF);
                ESP_LOGI(TAG, "ĐÈN → TẮT");
            }
            break;

        case IDX_BAT_QUAT:
            if (!s_quat_on) {
                s_quat_on = true;
                gpio_set_level((gpio_num_t)PIN_RELAY_QUAT, RELAY_ON);
                ESP_LOGI(TAG, "QUẠT → BẬT");
            }
            break;

        case IDX_TAT_QUAT:
            if (s_quat_on) {
                s_quat_on = false;
                gpio_set_level((gpio_num_t)PIN_RELAY_QUAT, RELAY_OFF);
                ESP_LOGI(TAG, "QUẠT → TẮT");
            }
            break;

        case IDX_BAT_HET:
            s_den_on = s_quat_on = true;
            gpio_set_level((gpio_num_t)PIN_RELAY_DEN,  RELAY_ON);
            gpio_set_level((gpio_num_t)PIN_RELAY_QUAT, RELAY_ON);
            ESP_LOGI(TAG, "BẬT HẾT");
            break;

        case IDX_TAT_HET:
            s_den_on = s_quat_on = false;
            gpio_set_level((gpio_num_t)PIN_RELAY_DEN,  RELAY_OFF);
            gpio_set_level((gpio_num_t)PIN_RELAY_QUAT, RELAY_OFF);
            ESP_LOGI(TAG, "TẮT HẾT");
            break;

        case IDX_UNKNOWN:
        default:
            // Không làm gì — không thay đổi trạng thái thiết bị
            return;
    }
    update_led();
}