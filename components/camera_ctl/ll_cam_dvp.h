#pragma once

#include "esp_err.h"
#include "esp_heap_caps.h"

#ifdef __cplusplus
extern "C" {
#endif

// Frame sizes for single-shot camera
typedef enum {
    FRAMESIZE_96X96,    // 96x96 for ML
    FRAMESIZE_QQVGA,    // 160x120
} framesize_t;

// Camera configuration
typedef struct {
    framesize_t frame_size;
    // Camera GPIO pins
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
} camera_config_t;

#include "esp32/rom/lldesc.h"

// Camera object structure
typedef struct {
    uint16_t width;
    uint16_t height;
    bool jpeg_mode;
    uint8_t fb_bytes_per_pixel;
    uint8_t in_bytes_per_pixel;
    int vsync_pin;  // VSYNC GPIO pin for interrupt
    
    uint8_t* dma_buffer;
    size_t dma_buffer_size;
    size_t dma_half_buffer_size;
    size_t dma_node_buffer_size;
    uint32_t dma_node_cnt;
    lldesc_t* dma;
    size_t frame_camera_bytes;   // bytes expected for one frame from camera (RGB565)
} cam_obj_t;

/**
 * @brief Initialize low-level camera DVP interface
 * @param config Camera configuration
 * @return ESP_OK on success
 */
esp_err_t ll_cam_dvp_init(const camera_config_t* config);

/**
 * @brief Capture a single frame to RGB buffer (VSYNC synchronized)
 * @param rgb_buffer Buffer to store RGB888 data
 * @param timeout_ms Timeout in milliseconds
 * @return ESP_OK on success
 */
esp_err_t ll_cam_dvp_capture(uint8_t* rgb_buffer, uint32_t timeout_ms);

/**
 * @brief Capture a single frame without VSYNC wait (for single-shot mode)
 * @param rgb_buffer Buffer to store RGB888 data
 * @param timeout_ms Timeout in milliseconds
 * @return ESP_OK on success
 */
esp_err_t ll_cam_dvp_capture_no_vsync(uint8_t* rgb_buffer, uint32_t timeout_ms);

/**
 * @brief Capture a single frame starting DMA first, then executing a trigger callback
 *        (used for true snapshot when a sensor trigger is needed).
 * @param rgb_buffer Buffer to store RGB565 bytes
 * @param timeout_ms Timeout in milliseconds
 * @param trigger_fn Function called immediately after DMA start to trigger snapshot
 * @return ESP_OK on success
 */
esp_err_t ll_cam_dvp_capture_with_trigger(uint8_t* rgb_buffer, uint32_t timeout_ms, esp_err_t (*trigger_fn)(void));

/**
 * @brief Deinitialize camera DVP interface
 */
void ll_cam_dvp_deinit();

#ifdef __cplusplus
}
#endif