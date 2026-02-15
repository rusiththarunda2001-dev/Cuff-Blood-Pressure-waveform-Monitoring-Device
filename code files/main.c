/*
 * SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "ui/ui.h"
#include "lvgl.h"

#include "hx710b.h"

static const char *TAG = "LVGL_EXAMPLE";
static const char *TAG3 = "hx710b sensor";
static const char *TAG2 = "MOTOR_CONTROL";

// IMPORTANT: Change these to the GPIOs you have connected the sensor to!
#define HX710B_DOUT_PIN GPIO_NUM_1
#define HX710B_SCK_PIN  GPIO_NUM_21

// Using SPI2 in the example
#define LCD_HOST  SPI2_HOST

// A good starting point is often around 0.8-0.95.
#define HPF_ALPHA 0.90f


#define EXAMPLE_LCD_PIXEL_CLOCK_HZ     (20 * 1000 * 1000)
#define EXAMPLE_LCD_BK_LIGHT_ON_LEVEL  1
#define EXAMPLE_LCD_BK_LIGHT_OFF_LEVEL !EXAMPLE_LCD_BK_LIGHT_ON_LEVEL
#define EXAMPLE_PIN_NUM_SCLK           4 // SPI Clock pin
#define EXAMPLE_PIN_NUM_MOSI           3  // SPI Master Out Slave In pin
#define EXAMPLE_PIN_NUM_MISO           -1 // SPI Master In Slave Out pin (not used)
#define EXAMPLE_PIN_NUM_LCD_DC         6 // LCD Data/Command pin
#define EXAMPLE_PIN_NUM_LCD_RST        7 // LCD Reset pin
#define EXAMPLE_PIN_NUM_LCD_CS         5 // LCD Chip Select pin
#define EXAMPLE_PIN_NUM_BK_LIGHT       -1 // LCD Backlight pin
#define EXAMPLE_PIN_NUM_TOUCH_CS       -1 // Touch screen Chip Select pin (if used)

// --- Pin Definitions ---
#define BUTTON_PIN1         8  // Start motor at max speed
#define BUTTON_PIN2         0  // Decrease motor speed
#define BUTTON_PIN3         20  // STOP motor
#define MOTOR_PIN_IN1       10  // Motor direction pin
#define MOTOR_PIN_ENABLE    2  // Motor PWM speed control pin

// --- PWM Properties ---
#define PWM_FREQ            30000 // 30 kHz
#define PWM_CHANNEL         LEDC_CHANNEL_0
#define PWM_TIMER           LEDC_TIMER_0
#define PWM_RESOLUTION      LEDC_TIMER_8_BIT // 8-bit resolution (0–255)


// --- Global State Variables ---
int dutyCycle = 255;
bool motorRunning = false;

// Previous button states for edge detection
int lastButtonState1 = 1; // 1 for HIGH
int lastButtonState2 = 1;
int lastButtonState3 = 1;

// The pixel number in horizontal and vertical directions
#define EXAMPLE_LCD_H_RES              128 
#define EXAMPLE_LCD_V_RES              160

// Bit number used to represent command and parameter
#define EXAMPLE_LCD_CMD_BITS           8
#define EXAMPLE_LCD_PARAM_BITS         8

// LVGL (Light and Versatile Graphics Library) configuration
#define EXAMPLE_LVGL_TICK_PERIOD_MS    2
#define EXAMPLE_LVGL_TASK_MAX_DELAY_MS 500
#define EXAMPLE_LVGL_TASK_MIN_DELAY_MS 1
#define EXAMPLE_LVGL_TASK_STACK_SIZE   (4 * 1024)
#define EXAMPLE_LVGL_TASK_PRIORITY     2



// Semaphore for LVGL mutex
static SemaphoreHandle_t lvgl_mux = NULL;

// Mutex to protect access to the shared sensor value
static SemaphoreHandle_t sensor_data_mutex;
// Shared variable to hold the latest sensor reading
static int32_t latest_sensor_value = 0;


// Notify LVGL that the flushing is done
static bool example_notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
    lv_disp_flush_ready(disp_driver);
    return false;
}

// LVGL callback to flush a specific area of the display
static void example_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
    // Copy a buffer's content to a specific area of the display
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
}

/* Rotate display and touch, when rotated screen in LVGL. Called when driver parameters are updated. */
static void example_lvgl_port_update_callback(lv_disp_drv_t *drv)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;

    switch (drv->rotated) {
    case LV_DISP_ROT_NONE:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, false, false);
        break;
    case LV_DISP_ROT_90:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, true, true);
        break;
    case LV_DISP_ROT_180:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, false, true);
        break;
    case LV_DISP_ROT_270:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, false, false);
        break;
    }
}

// Increase LVGL tick count by 2ms
static void example_increase_lvgl_tick(void *arg)
{
    lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}

// Lock LVGL mutex
bool example_lvgl_lock(int timeout_ms)
{
    // Convert timeout in milliseconds to FreeRTOS ticks
    // If `timeout_ms` is set to -1, the program will block until the condition is met
    const TickType_t timeout_ticks = (timeout_ms == -1) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTakeRecursive(lvgl_mux, timeout_ticks) == pdTRUE;
}

// Unlock LVGL mutex
void example_lvgl_unlock(void)
{
    xSemaphoreGiveRecursive(lvgl_mux);
}

// Task to handle LVGL operations
static void example_lvgl_port_task(void *arg)
{
    ESP_LOGI(TAG, "Starting LVGL task");
    uint32_t task_delay_ms = EXAMPLE_LVGL_TASK_MAX_DELAY_MS;
    while (1) {
        // Lock the mutex due to the LVGL APIs are not thread-safe
        if (example_lvgl_lock(-1)) {
            task_delay_ms = lv_timer_handler();
            // Release the mutex
            example_lvgl_unlock();
        }
        if (task_delay_ms > EXAMPLE_LVGL_TASK_MAX_DELAY_MS) {
            task_delay_ms = EXAMPLE_LVGL_TASK_MAX_DELAY_MS;
        } else if (task_delay_ms < EXAMPLE_LVGL_TASK_MIN_DELAY_MS) {
            task_delay_ms = EXAMPLE_LVGL_TASK_MIN_DELAY_MS;
        }
        vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
    }
}

void lvgl_unlock(SemaphoreHandle_t lvgl_mux)
{
    xSemaphoreGiveRecursive(lvgl_mux);
}

// Add this function to your main file (e.g., the one with app_main)

/**
 * Task dedicated to handling motor control via button inputs.
 */

static void motor_control_task(void *arg)
{
    ESP_LOGI(TAG2, "Motor control task started.");

    while (1) {
        // Read button states
        int buttonState1 = gpio_get_level(BUTTON_PIN1);
        int buttonState2 = gpio_get_level(BUTTON_PIN2);
        int buttonState3 = gpio_get_level(BUTTON_PIN3);

        // --- BUTTON 1: Start motor at max speed (dutyCycle = 255) ---
        if (buttonState1 == 0 && lastButtonState1 == 1) {
            dutyCycle = 255;
            ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL, dutyCycle);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL);
            gpio_set_level(MOTOR_PIN_IN1, 0); // Set motor direction
            motorRunning = true;
            ESP_LOGI(TAG2, "Motor started at max speed (dutyCycle = 255).");
        }
        lastButtonState1 = buttonState1;

        // --- BUTTON 2: Decrease speed by 5 per press ---
        if (buttonState2 == 0 && lastButtonState2 == 1) {
            if (dutyCycle > 5) {
                dutyCycle -= 5;
            } else {
                dutyCycle = 0;
            }
            ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL, dutyCycle);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL);
            ESP_LOGI(TAG2, "Motor speed decreased, dutyCycle = %d", dutyCycle);
        }
        lastButtonState2 = buttonState2;

        // --- BUTTON 3: Stop motor completely ---
        if (buttonState3 == 0 && lastButtonState3 == 1) {
            dutyCycle = 0;
            ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL, dutyCycle);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL);
            motorRunning = false;
            ESP_LOGI(TAG2, "Motor stopped.");
        }
        lastButtonState3 = buttonState3;

        // Add a small delay to prevent this task from hogging the CPU
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void hx710b_sensor_task(void *arg)
{
    hx710b_t sensor_handle = NULL;
    hx710b_config_t config = {
        .dout_pin = HX710B_DOUT_PIN,
        .pd_sck_pin = HX710B_SCK_PIN,
        .gain = HX710B_GAIN_128
    };

    esp_err_t err = hx710b_init(&config, &sensor_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG3, "HX710B initialization failed: %s", esp_err_to_name(err));
        vTaskDelete(NULL);
    }
    ESP_LOGI(TAG3, "HX710B initialized successfully.");

    // --- Variables for our High-Pass Filter ---
    static float hpf_filtered_value = 0.0f;
    static int32_t last_raw_value = 0;
    // ------------------------------------------

    // Read the first value to initialize the filter's state
    hx710b_read_average(sensor_handle, 1, &last_raw_value);

    while (1) {
        int32_t raw_value = 0;
        err = hx710b_read_average(sensor_handle, 1, &raw_value);

        if (err == ESP_OK) {
            // --- Apply the High-Pass Filter ---
            hpf_filtered_value = HPF_ALPHA * hpf_filtered_value + HPF_ALPHA * (raw_value - last_raw_value);
            last_raw_value = raw_value;
            // ----------------------------------

            ESP_LOGI(TAG3, "Raw: %ld, HPF Value: %.2f", raw_value, hpf_filtered_value);

            // Share the NEW filtered value
            if(xSemaphoreTake(sensor_data_mutex, portMAX_DELAY) == pdTRUE) {
                // Cast the float result to an integer for the chart
                latest_sensor_value = (int32_t)hpf_filtered_value;
                xSemaphoreGive(sensor_data_mutex);
            }
        } else {
            ESP_LOGE(TAG3, "Failed to read from sensor: %s", esp_err_to_name(err));
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
static void chart_update_task(void *arg)
{
    // // Wait a moment for the UI to be fully initialized
    // vTaskDelay(pdMS_TO_TICKS(1000));

    // // Add the series to the chart *once*
    if (example_lvgl_lock(-1)) {
        if (ui_Chart1) { // Check if chart object exists
             ui_Chart1_series_1 = lv_chart_add_series(ui_Chart1, lv_color_hex(0x20CA1D), LV_CHART_AXIS_PRIMARY_Y);
        }
        example_lvgl_unlock();
    }
    
    while (1) {
        int32_t current_sensor_value = 0;

        // 1. Safely read the latest sensor value from the shared variable
        if(xSemaphoreTake(sensor_data_mutex, portMAX_DELAY) == pdTRUE) {
            current_sensor_value = latest_sensor_value;
            xSemaphoreGive(sensor_data_mutex);
        }

        // 2. Lock the LVGL mutex and update the chart with the raw value
        if (example_lvgl_lock(-1)) {
            if (ui_Chart1 && ui_Chart1_series_1) {
                // Plot the raw value directly since scaling is handled by the UI
                lv_chart_set_next_value(ui_Chart1, ui_Chart1_series_1, current_sensor_value);
            }
            example_lvgl_unlock();
        }

        // 3. Wait before the next update
        vTaskDelay(pdMS_TO_TICKS(10)); // Update chart 5 times per second
    }
}

void app_main(void)
{
     // --- Configure GPIO pins ---
    // Configure motor direction pin as output
    gpio_set_direction(MOTOR_PIN_IN1, GPIO_MODE_OUTPUT);

    // Configure button pins as inputs with pull-ups
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BUTTON_PIN1) | (1ULL << BUTTON_PIN2) | (1ULL << BUTTON_PIN3),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
 

    
    // --- Configure LEDC (PWM) Timer ---
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = PWM_RESOLUTION,
        .timer_num        = PWM_TIMER,
        .freq_hz          = PWM_FREQ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    // --- Configure LEDC (PWM) Channel ---
    ledc_channel_config_t ledc_channel = {
        .gpio_num       = MOTOR_PIN_ENABLE,
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = PWM_CHANNEL,
        .timer_sel      = PWM_TIMER,
        .duty           = 0,
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel);

    ESP_LOGI(TAG2, "Motor button control initialized (8-bit PWM)");
    // The app_main can exit, but the pins will retain their configured state.
    // For more complex applications, you would typically have a loop here.

    static lv_disp_draw_buf_t disp_buf; // Contains internal graphic buffer(s) called draw buffer(s)
    static lv_disp_drv_t disp_drv;      // Contains callback functions

    // ... in your app_main ...
    if (EXAMPLE_PIN_NUM_BK_LIGHT >= 0) {
        ESP_LOGI(TAG, "Turn off LCD backlight");
        gpio_config_t bk_gpio_config = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = (1ULL << EXAMPLE_PIN_NUM_BK_LIGHT)
        };
        ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
    }

    ESP_LOGI(TAG, "Initialize SPI bus");
    spi_bus_config_t buscfg = {
        .sclk_io_num = EXAMPLE_PIN_NUM_SCLK,
        .mosi_io_num = EXAMPLE_PIN_NUM_MOSI,
        .miso_io_num = -1, // EXAMPLE_PIN_NUM_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = EXAMPLE_LCD_H_RES * 80 * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = EXAMPLE_PIN_NUM_LCD_DC,
        .cs_gpio_num = EXAMPLE_PIN_NUM_LCD_CS,
        .pclk_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = EXAMPLE_LCD_CMD_BITS,
        .lcd_param_bits = EXAMPLE_LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
        .on_color_trans_done = example_notify_lvgl_flush_ready,
        .user_ctx = &disp_drv,
    };
    // Attach the LCD to the SPI bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = EXAMPLE_PIN_NUM_LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };

    ESP_LOGI(TAG, "Install ST7789 panel driver");
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, false));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, false, true));

    // User can flush pre-defined pattern to the screen before we turn on the screen or backlight
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    ESP_LOGI(TAG, "Turn on LCD backlight");
    gpio_set_level(EXAMPLE_PIN_NUM_BK_LIGHT, EXAMPLE_LCD_BK_LIGHT_ON_LEVEL);

    ESP_LOGI(TAG, "Initialize LVGL library");
    lv_init();

    // Allocate draw buffers used by LVGL
    // It's recommended to choose the size of the draw buffer(s) to be at least 1/10 screen sized
    lv_color_t *buf1 = heap_caps_malloc(EXAMPLE_LCD_H_RES * 20 * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf1);
    lv_color_t *buf2 = heap_caps_malloc(EXAMPLE_LCD_H_RES * 20 * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf2);
    // Initialize LVGL draw buffers
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, EXAMPLE_LCD_H_RES * 20);

    ESP_LOGI(TAG, "Register display driver to LVGL");
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = EXAMPLE_LCD_H_RES;
    disp_drv.ver_res = EXAMPLE_LCD_V_RES;
    disp_drv.flush_cb = example_lvgl_flush_cb;
    disp_drv.drv_update_cb = example_lvgl_port_update_callback;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_handle;
    lv_disp_t *disp = lv_disp_drv_register(&disp_drv);

    ESP_LOGI(TAG, "Install LVGL tick timer");
    // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &example_increase_lvgl_tick,
        .name = "lvgl_tick"
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000));

    // Create LVGL mutex
    lvgl_mux = xSemaphoreCreateRecursiveMutex();
    assert(lvgl_mux);

    // ---> ADD THIS LINE <---
    sensor_data_mutex = xSemaphoreCreateMutex();
    assert(sensor_data_mutex); // Make sure it was created

    ESP_LOGI(TAG, "Create LVGL task");
    xTaskCreate(example_lvgl_port_task, "LVGL", EXAMPLE_LVGL_TASK_STACK_SIZE, NULL, EXAMPLE_LVGL_TASK_PRIORITY, NULL);

    ESP_LOGI(TAG, "Display LVGL Screen");
    // Lock the mutex due to the LVGL APIs are not thread-safe
    if (example_lvgl_lock(-1)) {
        ui_init(); // Initialize the UI
        // Release the mutex
        example_lvgl_unlock();
    }
    // ---> ADD THIS LINE TO START THE NEW TASK <---
    xTaskCreate(chart_update_task, "ChartUpdate", 4096, NULL, 5, NULL);
    // xTaskCreate(plot_chart, "Chart Task", 6144, NULL, configMAX_PRIORITIES-2, NULL);
    xTaskCreate(motor_control_task, "MotorControl", 4096, NULL, 5, NULL);
    xTaskCreate(hx710b_sensor_task, "HX710B_Sensor", 4096, NULL, 5, NULL);
} 
