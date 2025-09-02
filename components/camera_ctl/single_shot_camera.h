#pragma once

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Camera sensor types supported
 */
typedef enum {
    CAMERA_SENSOR_OV2640 = 0,
    CAMERA_SENSOR_OV3660 = 1
} camera_sensor_t;

/**
 * @brief Single-shot camera configuration
 */
typedef struct {
    // I2C configuration
    int sda_pin;
    int scl_pin;
    uint32_t i2c_freq;
    
    // Camera pins
    int pwdn_pin;
    int reset_pin;
    int xclk_pin;
    int frex_pin;     // Frame exposure trigger pin (for single-shot mode)
    int d0_pin;
    int d1_pin;
    int d2_pin;
    int d3_pin;
    int d4_pin;
    int d5_pin;
    int d6_pin;
    int d7_pin;
    int vsync_pin;
    int href_pin;
    int pclk_pin;
    
    // Image settings
    uint16_t width;
    uint16_t height;
    uint8_t jpeg_quality;
    uint32_t xclk_freq;
    
    // Sensor type
    camera_sensor_t sensor_type;
} single_shot_config_t;

/**
 * @brief Initialize single-shot camera system
 * @param config Camera configuration
 * @return ESP_OK on success, error code on failure
 */
esp_err_t single_shot_camera_init(const single_shot_config_t* config);

/**
 * @brief Capture a single JPEG image
 * 
 * This function:
 * 1. Powers up the OV2640 sensor
 * 2. Configures it for single-shot capture
 * 3. Triggers one frame capture
 * 4. Reads the JPEG data
 * 5. Powers down the sensor
 * 
 * @param jpeg_data Pointer to store JPEG data (caller must free)
 * @param jpeg_len Pointer to store JPEG data length
 * @return ESP_OK on success, error code on failure
 */
esp_err_t single_shot_capture_jpeg(uint8_t** jpeg_data, size_t* jpeg_len);

/**
 * @brief Capture a single RGB888 image (96x96 for ML)
 * 
 * This is optimized for your ML inference pipeline
 * 
 * @param rgb_data Pointer to store RGB888 data (96x96x3 bytes, caller must free)
 * @return ESP_OK on success, error code on failure
 */
esp_err_t single_shot_capture_rgb_96x96(uint8_t** rgb_data);

/**
 * @brief Trigger single-shot snapshot using FREX pin
 * @return ESP_OK on success, error code on failure
 */
esp_err_t single_shot_trigger_snapshot();

/**
 * @brief Trigger single-shot snapshot using I2C COM3 register (software method)
 * This method uses COM3[0] bit to enable single frame capture without FREX pin
 * @return ESP_OK on success, error code on failure
 */
esp_err_t single_shot_trigger_i2c_snapshot();

/**
 * @brief Reset sensor to continuous video mode after snapshot
 * This clears COM3[0] bit to return to live video output
 * @return ESP_OK on success, error code on failure
 */
esp_err_t single_shot_reset_to_continuous();

/**
 * @brief Check if camera is responsive
 * @return ESP_OK if camera responds, error code otherwise
 */
esp_err_t single_shot_camera_test();

/**
 * @brief Deinitialize single-shot camera system
 */
void single_shot_camera_deinit();

// Camera control functions extracted from ESP32-Camera OV2640 driver

/**
 * @brief White balance mode
 */
typedef enum {
    WB_MODE_AUTO = 0,
    WB_MODE_SUNNY = 1,
    WB_MODE_CLOUDY = 2,
    WB_MODE_OFFICE = 3,
    WB_MODE_HOME = 4
} wb_mode_t;

/**
 * @brief Set white balance mode
 * @param mode White balance mode
 * @return ESP_OK on success, error code on failure
 */
esp_err_t single_shot_set_wb_mode(wb_mode_t mode);

/**
 * @brief Set brightness (-2 to +2)
 * @param level Brightness level
 * @return ESP_OK on success, error code on failure  
 */
esp_err_t single_shot_set_brightness(int level);

/**
 * @brief Set contrast (-2 to +2)
 * @param level Contrast level
 * @return ESP_OK on success, error code on failure
 */
esp_err_t single_shot_set_contrast(int level);

/**
 * @brief Set saturation (-2 to +2)
 * @param level Saturation level
 * @return ESP_OK on success, error code on failure
 */
esp_err_t single_shot_set_saturation(int level);

/**
 * @brief Set AGC gain (0 to 30)
 * @param gain_db Gain in dB
 * @return ESP_OK on success, error code on failure
 */
esp_err_t single_shot_set_agc_gain(int gain_db);

/**
 * @brief Set AE (auto exposure) level (-2 to +2)
 * @param level AE level
 * @return ESP_OK on success, error code on failure
 */
esp_err_t single_shot_set_ae_level(int level);

/**
 * @brief Set frame size
 * @param width Image width
 * @param height Image height
 * @return ESP_OK on success, error code on failure
 */
esp_err_t single_shot_set_framesize(uint16_t width, uint16_t height);

/**
 * @brief Default configuration for AI-Thinker ESP32-CAM
 */
#define SINGLE_SHOT_CONFIG_DEFAULT() { \
    .sda_pin = 26, \
    .scl_pin = 27, \
    .i2c_freq = 50000, \
    .pwdn_pin = 32, \
    .reset_pin = -1, \
    .xclk_pin = 0, \
    .frex_pin = 4, \
    .d0_pin = 5, \
    .d1_pin = 18, \
    .d2_pin = 19, \
    .d3_pin = 21, \
    .d4_pin = 36, \
    .d5_pin = 39, \
    .d6_pin = 34, \
    .d7_pin = 35, \
    .vsync_pin = 25, \
    .href_pin = 23, \
    .pclk_pin = 22, \
    .width = 96, \
    .height = 96, \
    .jpeg_quality = 12, \
    .xclk_freq = 10000000, \
    .sensor_type = CAMERA_SENSOR_OV2640 \
}

// Configuration for Seeed Studio XIAO ESP32-S3 Sense with OV3660
#define SINGLE_SHOT_CONFIG_SEEED_OV3660() { \
    .sda_pin = 5, \
    .scl_pin = 6, \
    .i2c_freq = 100000, \
    .pwdn_pin = -1, \
    .reset_pin = -1, \
    .xclk_pin = 10, \
    .frex_pin = 9, \
    .d0_pin = 15, \
    .d1_pin = 17, \
    .d2_pin = 18, \
    .d3_pin = 16, \
    .d4_pin = 14, \
    .d5_pin = 12, \
    .d6_pin = 11, \
    .d7_pin = 48, \
    .vsync_pin = 38, \
    .href_pin = 47, \
    .pclk_pin = 13, \
    .width = 96, \
    .height = 96, \
    .jpeg_quality = 12, \
    .xclk_freq = 20000000, \
    .sensor_type = CAMERA_SENSOR_OV3660 \
}

#ifdef __cplusplus
}
#endif