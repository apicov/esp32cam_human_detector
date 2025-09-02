#pragma once
#include <stdint.h>//lib for ints i-e int8_t upto 1int64_t
#include <stdio.h> // macros,input output, files etc
#include <esp_log.h>


void resize_color_image(uint8_t *src, int srcWidth, int srcHeight, 
                      uint8_t *dst, int dstWidth, int dstHeight);

void saveAsPPM(const char *filename, uint8_t *image, int width, int height);

/**
 * @brief Save an RGB888 image to PPM with optional channel swap and vertical flip.
 * @param filename Destination path
 * @param image Source buffer (RGB888, width*height*3 bytes)
 * @param width Image width
 * @param height Image height
 * @param swap_rb If true, swap red and blue channels when saving
 * @param flip_vertical If true, save with rows reversed (bottom-up)
 * @return true on success, false on error
 */
bool saveAsPPMEx(const char *filename, const uint8_t *image, int width, int height,
                 bool swap_rb, bool flip_vertical);

void resize_color_image_bilinear(uint8_t* src_image, int src_width, int src_height, 
                      uint8_t* dst_image, int dst_width, int dst_height);