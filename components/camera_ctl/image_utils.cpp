#include "image_utils.h"


void resize_color_image(uint8_t *src, int srcWidth, int srcHeight, 
                      uint8_t *dst, int dstWidth, int dstHeight) 
{
    for (int y = 0; y < dstHeight; y++) {
        for (int x = 0; x < dstWidth; x++) {
            // Map destination coordinates to source coordinates
            int srcX = x * srcWidth / dstWidth;
            int srcY = y * srcHeight / dstHeight;

            // Calculate source index in 24-bit array (RGB888)
            int srcIndex = (srcY * srcWidth + srcX) * 3; // RGB888 = 3 bytes per pixel

            // Extract RGB components
            // swap r and b channels (because of bug in fmt2rgb888 function)
            uint8_t b = src[srcIndex];
            uint8_t g = src[srcIndex + 1];
            uint8_t r = src[srcIndex + 2];

            // Write to the destination array
            int dstIndex = (y * dstWidth + x) * 3; // RGB888 = 3 bytes per pixel
            dst[dstIndex] = r;
            dst[dstIndex + 1] = g;
            dst[dstIndex + 2] = b;
        }
    }
}


void resize_color_image_bilinear(uint8_t* src_image, int src_width, int src_height, 
                      uint8_t* dst_image, int dst_width, int dst_height) 
{
    // Calculate scaling factors
    float x_ratio = (float)(src_width - 1) / (dst_width - 1);
    float y_ratio = (float)(src_height - 1) / (dst_height - 1);
    
    for (int y = 0; y < dst_height; y++) {
        for (int x = 0; x < dst_width; x++) {
            // Calculate source pixel coordinates
            float src_x = x * x_ratio;
            float src_y = y * y_ratio;
            
            // Get integer and fractional parts
            int x1 = (int)src_x;
            int y1 = (int)src_y;
            int x2 = x1 + 1;
            int y2 = y1 + 1;
            
            // Ensure we don't go out of bounds
            x2 = (x2 < src_width) ? x2 : src_width - 1;
            y2 = (y2 < src_height) ? y2 : src_height - 1;
            
            // Calculate interpolation weights
            float wx = src_x - x1;
            float wy = src_y - y1;
            
            // Interpolate for each color channel
            for (int c = 0; c < 3; c++) {
                // Get surrounding pixel values
                uint8_t p11 = src_image[(y1 * src_width + x1) * 3 + c];
                uint8_t p12 = src_image[(y1 * src_width + x2) * 3 + c];
                uint8_t p21 = src_image[(y2 * src_width + x1) * 3 + c];
                uint8_t p22 = src_image[(y2 * src_width + x2) * 3 + c];
                
                // Bilinear interpolation
                float interpolated = 
                    p11 * (1 - wx) * (1 - wy) +
                    p12 * wx * (1 - wy) +
                    p21 * (1 - wx) * wy +
                    p22 * wx * wy;
                
                // Store interpolated pixel
                dst_image[(y * dst_width + x) * 3 + c] = (uint8_t)interpolated;
            }
        }
    }
    
}


// Function to save an RGB888 image as a PPM file
void saveAsPPM(const char *filename, uint8_t *image, int width, int height) {
    // Open the file in binary write mode
    FILE *file = fopen(filename, "wb");
    if (!file) {
        ESP_LOGE("PPM","Failed to open file: %s", filename);
        return;
    }

    // Write the PPM header
    fprintf(file, "P6\n%d %d\n255\n", width, height);

    // Calculate the total number of pixels
    int totalPixels = width * height;

    // Write the RGB data to the file
    if (fwrite(image, 1, totalPixels * 3, file) != totalPixels * 3) {
        ESP_LOGE("PPM","Error writing image data to file");
    }

    // Close the file
    fclose(file);

    ESP_LOGI("PPM", "PPM image saved successfully: %s", filename);
}