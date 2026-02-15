
#include <stdlib.h>
#include "hx710b.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "rom/ets_sys.h" // For ets_delay_us

static const char *TAG = "hx710b";

// Internal state struct
struct hx710b_s {
    gpio_num_t dout_pin;
    gpio_num_t pd_sck_pin;
    hx710b_gain_t gain;
    int32_t offset;
    float scale;
};

// The ESP32 is a fast CPU, so we need to add delays to the bit-banging
static uint8_t shift_in_byte(gpio_num_t data_pin, gpio_num_t clock_pin) {
    uint8_t value = 0;
    for (int i = 0; i < 8; ++i) {
        gpio_set_level(clock_pin, 1);
        ets_delay_us(1);
        value |= gpio_get_level(data_pin) << (7 - i);
        gpio_set_level(clock_pin, 0);
        ets_delay_us(1);
    }
    return value;
}

esp_err_t hx710b_init(const hx710b_config_t* config, hx710b_t* handle_out) {
    if (!config || !handle_out) {
        return ESP_ERR_INVALID_ARG;
    }

    hx710b_t handle = (hx710b_t)calloc(1, sizeof(struct hx710b_s));
    if (!handle) {
        ESP_LOGE(TAG, "Failed to allocate memory for handle");
        return ESP_ERR_NO_MEM;
    }

    handle->dout_pin = config->dout_pin;
    handle->pd_sck_pin = config->pd_sck_pin;
    handle->gain = config->gain;
    handle->offset = 0;
    handle->scale = 1.0f;

    // Configure SCK pin as output
    gpio_config_t sck_conf = {
        .pin_bit_mask = (1ULL << handle->pd_sck_pin),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&sck_conf);

    // Configure DOUT pin as input
    gpio_config_t dout_conf = {
        .pin_bit_mask = (1ULL << handle->dout_pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&dout_conf);

    // Power up the chip
    gpio_set_level(handle->pd_sck_pin, 0);

    *handle_out = handle;
    ESP_LOGI(TAG, "HX710B initialized on DOUT=%d, SCK=%d", handle->dout_pin, handle->pd_sck_pin);
    return ESP_OK;
}

esp_err_t hx710b_deinit(hx710b_t handle) {
    if (!handle) return ESP_ERR_INVALID_ARG;
    free(handle);
    return ESP_OK;
}

bool hx710b_is_ready(hx710b_t handle) {
    return gpio_get_level(handle->dout_pin) == 0;
}

void hx710b_wait_ready(hx710b_t handle, uint32_t delay_ms) {
    while (!hx710b_is_ready(handle)) {
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}

bool hx710b_wait_ready_timeout(hx710b_t handle, uint32_t timeout_ms) {
    int64_t start_time = esp_timer_get_time();
    while ((esp_timer_get_time() - start_time) / 1000 < timeout_ms) {
        if (hx710b_is_ready(handle)) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return false;
}

esp_err_t hx710b_read_raw(hx710b_t handle, int32_t* data) {
    if (!hx710b_wait_ready_timeout(handle, 500)) {
        ESP_LOGW(TAG, "Sensor not ready, timeout.");
        return ESP_ERR_TIMEOUT;
    }

    uint8_t data_bytes[3];

    // --- Start of critical section ---
    // Use a task-based critical section to prevent context switches
    // during the sensitive bit-banging process.
    portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
    portENTER_CRITICAL(&mux);

    // Pulse the clock pin 24 times to read the data
    data_bytes[2] = shift_in_byte(handle->dout_pin, handle->pd_sck_pin); // MSB
    data_bytes[1] = shift_in_byte(handle->dout_pin, handle->pd_sck_pin);
    data_bytes[0] = shift_in_byte(handle->dout_pin, handle->pd_sck_pin); // LSB

    // Set the gain for the next reading by pulsing the clock pin
    for (int i = 0; i < handle->gain; i++) {
        gpio_set_level(handle->pd_sck_pin, 1);
        ets_delay_us(1);
        gpio_set_level(handle->pd_sck_pin, 0);
        ets_delay_us(1);
    }

    portEXIT_CRITICAL(&mux);
    // --- End of critical section ---

    // Replicate the most significant bit to pad out a 32-bit signed integer
    uint32_t value = 0;
    if (data_bytes[2] & 0x80) {
        value = 0xFF000000;
    }

    value |= ((uint32_t)data_bytes[2] << 16);
    value |= ((uint32_t)data_bytes[1] << 8);
    value |= ((uint32_t)data_bytes[0]);

    *data = (int32_t)value;
    return ESP_OK;
}

esp_err_t hx710b_read_average(hx710b_t handle, uint8_t times, int32_t* data) {
    int64_t sum = 0;
    int32_t temp_data;
    for (uint8_t i = 0; i < times; i++) {
        esp_err_t err = hx710b_read_raw(handle, &temp_data);
        if (err != ESP_OK) return err;
        sum += temp_data;
    }
    *data = sum / times;
    return ESP_OK;
}

esp_err_t hx710b_tare(hx710b_t handle, uint8_t times) {
    int32_t average_val;
    esp_err_t err = hx710b_read_average(handle, times, &average_val);
    if (err == ESP_OK) {
        handle->offset = average_val;
        ESP_LOGI(TAG, "Tare complete. Offset set to: %ld", handle->offset);
    }
    return err;
}

void hx710b_set_offset(hx710b_t handle, int32_t offset) {
    handle->offset = offset;
}

int32_t hx710b_get_offset(hx710b_t handle) {
    return handle->offset;
}

void hx710b_set_scale(hx710b_t handle, float scale) {
    handle->scale = scale;
}

float hx710b_get_scale(hx710b_t handle) {
    return handle->scale;
}

esp_err_t hx710b_get_units(hx710b_t handle, uint8_t times, float* units) {
    int32_t raw_data;
    esp_err_t err = hx710b_read_average(handle, times, &raw_data);
    if (err == ESP_OK) {
        if (handle->scale == 0) {
            *units = 0;
            return ESP_FAIL; // Avoid division by zero
        }
        *units = (float)(raw_data - handle->offset) / handle->scale;
    }
    return err;
}