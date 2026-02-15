#ifndef HX710B_H
#define HX710B_H

#include "driver/gpio.h"
#include "esp_err.h"

// Opaque handle for the HX710B sensor
typedef struct hx710b_s* hx710b_t;

// Define gain values
typedef enum {
    HX710B_GAIN_128 = 1, // Channel A, gain factor 128
    HX710B_GAIN_32  = 2, // Channel B, gain factor 32
    HX710B_GAIN_64  = 3  // Channel A, gain factor 64
} hx710b_gain_t;

/**
 * @brief Configuration struct for HX710B initialization
 */
typedef struct {
    gpio_num_t dout_pin;    // Data output pin
    gpio_num_t pd_sck_pin;  // Power down and serial clock input pin
    hx710b_gain_t gain;     // Gain and channel selection
} hx710b_config_t;

/**
 * @brief Initialize the HX710B sensor
 *
 * @param config Pointer to the configuration struct
 * @param handle_out Pointer to store the created handle
 * @return ESP_OK on success, or an error code on failure
 */
esp_err_t hx710b_init(const hx710b_config_t* config, hx710b_t* handle_out);

/**
 * @brief Deinitialize the HX710B sensor and free resources
 *
 * @param handle The sensor handle
 * @return ESP_OK on success
 */
esp_err_t hx710b_deinit(hx710b_t handle);

/**
 * @brief Check if data is ready to be read from the sensor
 *
 * @param handle The sensor handle
 * @return true if data is ready, false otherwise
 */
bool hx710b_is_ready(hx710b_t handle);

/**
 * @brief Wait for the chip to become ready
 *
 * @note This is a blocking call.
 *
 * @param handle The sensor handle
 * @param delay_ms Delay between checks
 */
void hx710b_wait_ready(hx710b_t handle, uint32_t delay_ms);

/**
 * @brief Wait for the chip to become ready, with a timeout
 *
 * @param handle The sensor handle
 * @param timeout_ms Timeout in milliseconds
 * @return true if the chip became ready, false on timeout
 */
bool hx710b_wait_ready_timeout(hx710b_t handle, uint32_t timeout_ms);

/**
 * @brief Read a raw 24-bit value from the HX710B
 *
 * @param handle The sensor handle
 * @param data Pointer to store the read value
 * @return ESP_OK on success, ESP_ERR_TIMEOUT if not ready
 */
esp_err_t hx710b_read_raw(hx710b_t handle, int32_t* data);

/**
 * @brief Read an averaged raw value from the HX710B
 *
 * @param handle The sensor handle
 * @param times Number of readings to average
 * @param data Pointer to store the averaged value
 * @return ESP_OK on success
 */
esp_err_t hx710b_read_average(hx710b_t handle, uint8_t times, int32_t* data);

/**
 * @brief Tare the scale: set the current reading as zero offset
 *
 * @param handle The sensor handle
 * @param times Number of readings to average for taring
 * @return ESP_OK on success
 */
esp_err_t hx710b_tare(hx710b_t handle, uint8_t times);

/**
 * @brief Set the offset value
 *
 * @param handle The sensor handle
 * @param offset The offset value
 */
void hx710b_set_offset(hx710b_t handle, int32_t offset);

/**
 * @brief Get the current offset value
 *
 * @param handle The sensor handle
 * @return The offset value
 */
int32_t hx710b_get_offset(hx710b_t handle);

/**
 * @brief Set the scale factor for converting raw value to units
 *
 * @param handle The sensor handle
 * @param scale The scale factor
 */
void hx710b_set_scale(hx710b_t handle, float scale);

/**
 * @brief Get the current scale factor
 *
 * @param handle The sensor handle
 * @return The scale factor
 */
float hx710b_get_scale(hx710b_t handle);

/**
 * @brief Get the value in units (raw_value - offset) / scale
 *
 * @param handle The sensor handle
 * @param times Number of readings to average
 * @param units Pointer to store the value in units
 * @return ESP_OK on success
 */
esp_err_t hx710b_get_units(hx710b_t handle, uint8_t times, float* units);

#endif // HX710B_H
