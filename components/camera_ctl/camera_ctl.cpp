// ESP
#include <esp_log.h>
#include <esp_check.h>
#include <esp_heap_caps.h>
#include "camera_ctl.hpp"
#if CONFIG_SNAP_SINGLE_SHOT_LL_DVP
#include "single_shot_camera.h"
#include "ll_cam_dvp.h"
#else
#include "esp_camera.h"
#include "img_converters.h"
#endif

CameraCtl::CameraCtl() : initialized(false)
{
    ESP_LOGI(TAG, "=== Initializing Camera ===");
    ESP_LOGI(TAG, "Free PSRAM: %zu bytes", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    ESP_LOGI(TAG, "Free internal RAM: %zu bytes", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));

#if CONFIG_SNAP_SINGLE_SHOT_LL_DVP
    single_shot_config_t cfg = SINGLE_SHOT_CONFIG_DEFAULT();
    // Use 20MHz XCLK for stability similar to esp_camera path
    cfg.xclk_freq = 20000000;
    esp_err_t ret = single_shot_camera_init(&cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Single-shot camera init failed: %s", esp_err_to_name(ret));
        initialized = false;
    } else {
        ESP_LOGI(TAG, "Single-shot camera initialized");
        initialized = true;
    }
#else
    // ESP32-CAM pin configuration
    camera_config_t config = {
        .pin_pwdn = 32,
        .pin_reset = -1,
        .pin_xclk = 0,
        .pin_sccb_sda = 26,
        .pin_sccb_scl = 27,
        .pin_d7 = 35,
        .pin_d6 = 34,
        .pin_d5 = 39,
        .pin_d4 = 36,
        .pin_d3 = 21,
        .pin_d2 = 19,
        .pin_d1 = 18,
        .pin_d0 = 5,
        .pin_vsync = 25,
        .pin_href = 23,
        .pin_pclk = 22,
        
        .xclk_freq_hz = 20000000,
        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,
        
        .pixel_format = PIXFORMAT_RGB565,
        .frame_size = FRAMESIZE_96X96,
        
        .jpeg_quality = 12,
        .fb_count = 1,
        .fb_location = CAMERA_FB_IN_PSRAM,
        .grab_mode = CAMERA_GRAB_LATEST
    };
    
    esp_err_t ret = esp_camera_init(&config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ESP32-Camera initialization failed: %s", esp_err_to_name(ret));
        initialized = false;
    } else {
        ESP_LOGI(TAG, "ESP32-Camera initialized successfully");
        initialized = true;
    }
#endif
}


bool CameraCtl::is_initialized() const
{
    return initialized;
}

void CameraCtl::capture_do(std::function<void(const Picture &)> f)
{
    if (!initialized) {
        ESP_LOGE(TAG, "Camera not initialized, cannot capture");
        return;
    }
    
    Picture p{};  // Constructor captures one frame (LL-DVP when enabled)
    
    // Check if capture was successful
    if (p.image() == nullptr) {
        ESP_LOGE(TAG, "Camera capture failed - RGB data is null");
        return;
    }
    
    f(p);
    // Destructor will free the RGB data
}




/* CameraCtl::Picture */
/* ================== */
CameraCtl::Picture::Picture() : rgb_data(nullptr), owns_data(false)
{
#if CONFIG_SNAP_SINGLE_SHOT_LL_DVP
    ESP_LOGI(TAG, "Capturing frame (single-shot LL-DVP)...");

    // Capture RGB565 raw data first, then convert to RGB888
    const int width = 96;
    const int height = 96;
    const size_t rgb565_size = width * height * 2;
    uint8_t* rgb565 = (uint8_t*)malloc(rgb565_size);
    if (!rgb565) {
        ESP_LOGE(TAG, "Failed to allocate RGB565 buffer");
        return;
    }

    esp_err_t cap = ll_cam_dvp_capture(rgb565, 3000);
    if (cap != ESP_OK) {
        ESP_LOGE(TAG, "LL-DVP capture failed: %s", esp_err_to_name(cap));
        free(rgb565);
        return;
    }

    rgb_data = (uint8_t*)malloc(width * height * 3);
    if (!rgb_data) {
        ESP_LOGE(TAG, "Failed to allocate RGB888 buffer");
        free(rgb565);
        return;
    }

    convert_rgb565_to_rgb888(rgb565, rgb_data, width, height);
    owns_data = true;
    free(rgb565);
#else
    ESP_LOGI(TAG, "Capturing frame using ESP32-Camera...");
    
    // Capture frame using ESP32-Camera driver
    fb = esp_camera_fb_get();
    
    if (fb != nullptr) {
        ESP_LOGI(TAG, "Frame captured: %dx%d, format=%d, len=%zu", 
                 fb->width, fb->height, fb->format, fb->len);
        
        if (fb->format == PIXFORMAT_RGB565 || fb->format == PIXFORMAT_YUV422 || fb->format == PIXFORMAT_GRAYSCALE) {
            // Convert to RGB888 using esp32-camera's img_converters
            rgb_data = (uint8_t*)malloc(fb->width * fb->height * 3);
            if (rgb_data) {
                if (!fmt2rgb888(fb->buf, fb->len, (pixformat_t)fb->format, rgb_data)) {
                    ESP_LOGE(TAG, "fmt2rgb888 failed (format=%d)", fb->format);
                    free(rgb_data);
                    rgb_data = nullptr;
                } else {
                    owns_data = true;
                    ESP_LOGI(TAG, "fmt2rgb888 conversion complete");
                }
            } else {
                ESP_LOGE(TAG, "Failed to allocate RGB888 buffer");
            }
        } else {
            ESP_LOGW(TAG, "Unexpected pixel format: %d", fb->format);
        }
    } else {
        ESP_LOGE(TAG, "Camera capture failed - no framebuffer");
    }
#endif
}

CameraCtl::Picture::~Picture()
{
    if (rgb_data && owns_data) {
        free(rgb_data);
        ESP_LOGD(TAG, "Freed RGB888 conversion buffer");
    }

#if !CONFIG_SNAP_SINGLE_SHOT_LL_DVP
    if (fb) {
        esp_camera_fb_return(fb);
        ESP_LOGD(TAG, "Returned camera framebuffer");
    }
#endif
}

const uint8_t *CameraCtl::Picture::image() const
{
    return rgb_data;  // Return RGB888 data for ML
}

size_t CameraCtl::Picture::size() const
{
    return rgb_data ? (96 * 96 * 3) : 0;  // RGB888 format
}

uint16_t CameraCtl::Picture::width() const
{
    return fb ? fb->width : 0;
}

uint16_t CameraCtl::Picture::height() const
{
    return fb ? fb->height : 0;
}

// Helper function to convert RGB565 to RGB888
void CameraCtl::Picture::convert_rgb565_to_rgb888(const uint8_t* rgb565, uint8_t* rgb888, int width, int height) {
    for (int i = 0; i < width * height; i++) {
        uint16_t pixel = (rgb565[i*2 + 1] << 8) | rgb565[i*2];  // Little endian
        
        // Extract RGB565 components
        uint8_t r = (pixel >> 11) & 0x1F;  // 5 bits
        uint8_t g = (pixel >> 5) & 0x3F;   // 6 bits  
        uint8_t b = pixel & 0x1F;          // 5 bits
        
        // Convert to RGB888
        rgb888[i*3 + 0] = (r << 3) | (r >> 2);  // R: 5->8 bits
        rgb888[i*3 + 1] = (g << 2) | (g >> 4);  // G: 6->8 bits
        rgb888[i*3 + 2] = (b << 3) | (b >> 2);  // B: 5->8 bits
    }
}
