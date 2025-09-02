#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Convert JPEG data to RGB888 format
 * 
 * @param jpeg_data Pointer to JPEG data
 * @param jpeg_len Length of JPEG data in bytes  
 * @param rgb_buf Output buffer for RGB888 data (must be pre-allocated)
 * @param max_width Maximum expected width (for buffer size validation)
 * @param max_height Maximum expected height (for buffer size validation)
 * @param actual_width Pointer to store actual decoded width
 * @param actual_height Pointer to store actual decoded height
 * @return true on success, false on failure
 */
bool jpeg_to_rgb888(const uint8_t *jpeg_data, size_t jpeg_len, 
                    uint8_t *rgb_buf, uint16_t max_width, uint16_t max_height,
                    uint16_t *actual_width, uint16_t *actual_height);

/**
 * @brief Convert YUV pixel to RGB
 * @param y Y component (luminance)
 * @param u U component (chrominance)
 * @param v V component (chrominance)
 * @param r Pointer to store red component
 * @param g Pointer to store green component  
 * @param b Pointer to store blue component
 */
void yuv2rgb(uint8_t y, uint8_t u, uint8_t v, uint8_t *r, uint8_t *g, uint8_t *b);

/**
 * @brief Convert RGB565 pixel to RGB888
 * @param rgb565 Input RGB565 value (16-bit)
 * @param r Pointer to store red component (8-bit)
 * @param g Pointer to store green component (8-bit)
 * @param b Pointer to store blue component (8-bit)
 */
void rgb565_to_rgb888(uint16_t rgb565, uint8_t *r, uint8_t *g, uint8_t *b);

/**
 * @brief Convert RGB888 pixel to RGB565
 * @param r Red component (8-bit)
 * @param g Green component (8-bit)
 * @param b Blue component (8-bit)
 * @return RGB565 value (16-bit)
 */
uint16_t rgb888_to_rgb565(uint8_t r, uint8_t g, uint8_t b);

#ifdef __cplusplus
}
#endif