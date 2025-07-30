#pragma once
#include <stdint.h>//lib for ints i-e int8_t upto 1int64_t
#include <stdio.h> // macros,input output, files etc
#include <esp_log.h>


void resize_color_image(uint8_t *src, int srcWidth, int srcHeight, 
                      uint8_t *dst, int dstWidth, int dstHeight);

void saveAsPPM(const char *filename, uint8_t *image, int width, int height);

void resize_color_image_bilinear(uint8_t* src_image, int src_width, int src_height, 
                      uint8_t* dst_image, int dst_width, int dst_height);