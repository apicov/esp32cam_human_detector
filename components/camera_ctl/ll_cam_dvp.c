// Low-level camera capture implementation
// Adapted from ESP32-Camera library for single-shot operation

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "soc/i2s_struct.h"
#include "soc/i2s_reg.h"
#include "soc/gpio_sig_map.h"
#include "hal/gpio_ll.h"
#include "driver/gpio.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_gpio.h"
#include "esp32/rom/lldesc.h"
#include "ll_cam_dvp.h"

#define gpio_matrix_in(a,b,c) esp_rom_gpio_connect_in_signal(a,b,c)

static const char *TAG = "ll_cam_dvp";

#define I2S_ISR_ENABLE(i) {I2S0.int_clr.i = 1;I2S0.int_ena.i = 1;}
#define I2S_ISR_DISABLE(i) {I2S0.int_ena.i = 0;I2S0.int_clr.i = 1;}

typedef union {
    struct {
        uint32_t sample2:8;
        uint32_t unused2:8;
        uint32_t sample1:8;
        uint32_t unused1:8;
    };
    uint32_t val;
} dma_elem_t;

typedef enum {
    /* camera sends byte sequence: s1, s2, s3, s4, ...
     * fifo receives: 00 s1 00 s2, 00 s2 00 s3, 00 s3 00 s4, ...
     */
    SM_0A0B_0B0C = 0,
    /* camera sends byte sequence: s1, s2, s3, s4, ...
     * fifo receives: 00 s1 00 s2, 00 s3 00 s4, ...
     */
    SM_0A0B_0C0D = 1,
    /* camera sends byte sequence: s1, s2, s3, s4, ...
     * fifo receives: 00 s1 00 00, 00 s2 00 00, 00 s3 00 00, ...
     */
    SM_0A00_0B00 = 3,
} i2s_sampling_mode_t;

// Global camera object
static cam_obj_t s_cam_obj = {};
static bool s_cam_initialized = false;
static SemaphoreHandle_t s_frame_ready = NULL;
static intr_handle_t s_i2s_intr_handle = NULL;
static bool s_vsync_detected = false;

// DMA filter to compact valid data bytes from I2S DMA stream into contiguous RGB565 bytes
static size_t IRAM_ATTR ll_cam_dma_filter_rgb(uint8_t* dst, const uint8_t* src, size_t len) {
    const dma_elem_t* dma_elem = (const dma_elem_t*)src;
    size_t end = len / sizeof(dma_elem_t);
    size_t olen = 0;
    
    for (size_t i = 0; i < end; ++i) {
        dst[olen++] = dma_elem[i].sample1;
        dst[olen++] = dma_elem[i].sample2;
    }
    return olen; // olen should be exactly half of len
}

// VSYNC GPIO interrupt handler
static void IRAM_ATTR ll_cam_vsync_gpio_isr(void* arg) {
    s_vsync_detected = true;
    // Can't use ESP_LOGI in ISR, but can increment a counter for debugging
}

// I2S DMA interrupt handler
static void IRAM_ATTR ll_cam_dma_isr(void* arg) {
    (void)arg;  // Suppress unused parameter warning
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    I2S0.int_clr.in_suc_eof = 1;
    if (s_frame_ready) {
        xSemaphoreGiveFromISR(s_frame_ready, &xHigherPriorityTaskWoken);
    }
    
    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

// Configure GPIO matrix to route camera pins to I2S peripheral
static esp_err_t configure_camera_gpio(const camera_config_t* config) {
    ESP_LOGD(TAG, "Configuring camera GPIO pins");
    
    // Configure pixel clock (PCLK)
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[config->pclk_pin], PIN_FUNC_GPIO);
    gpio_set_direction(config->pclk_pin, GPIO_MODE_INPUT);
    gpio_set_pull_mode(config->pclk_pin, GPIO_FLOATING);
    gpio_matrix_in(config->pclk_pin, I2S0I_WS_IN_IDX, false);
    
    // Configure VSYNC with GPIO interrupt
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[config->vsync_pin], PIN_FUNC_GPIO);
    gpio_set_direction(config->vsync_pin, GPIO_MODE_INPUT);
    gpio_set_pull_mode(config->vsync_pin, GPIO_PULLUP_ONLY);
    
    // Set up VSYNC interrupt (try any edge to detect any transition)
    gpio_config_t vsync_conf = {
        .intr_type = GPIO_INTR_ANYEDGE,
        .pin_bit_mask = 1ULL << config->vsync_pin,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1,
        .pull_down_en = 0
    };
    gpio_config(&vsync_conf);
    
    // Don't connect to I2S - we'll handle VSYNC via GPIO interrupt
    // gpio_matrix_in(config->vsync_pin, I2S0I_V_SYNC_IDX, false);
    
    // Configure HREF
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[config->href_pin], PIN_FUNC_GPIO);
    gpio_set_direction(config->href_pin, GPIO_MODE_INPUT);
    gpio_set_pull_mode(config->href_pin, GPIO_FLOATING);
    gpio_matrix_in(config->href_pin, I2S0I_H_SYNC_IDX, false);
    
    // Configure data pins D0-D7
    int data_pins[8] = {
        config->d0_pin, config->d1_pin, config->d2_pin, config->d3_pin,
        config->d4_pin, config->d5_pin, config->d6_pin, config->d7_pin
    };
    
    for (int i = 0; i < 8; i++) {
        PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[data_pins[i]], PIN_FUNC_GPIO);
        gpio_set_direction(data_pins[i], GPIO_MODE_INPUT);
        gpio_set_pull_mode(data_pins[i], GPIO_FLOATING);
        gpio_matrix_in(data_pins[i], I2S0I_DATA_IN0_IDX + i, false);
    }
    
    // Configure H_ENABLE signal (fixed value from ESP32-Camera)
    gpio_matrix_in(0x38, I2S0I_H_ENABLE_IDX, false);
    
    ESP_LOGD(TAG, "Camera GPIO configuration complete");
    return ESP_OK;
}

// Allocate and configure DMA descriptors
static lldesc_t* allocate_dma_descriptors(uint32_t count, size_t size, uint8_t* buffer) {
    ESP_LOGD(TAG, "Allocating %u DMA descriptors, size %zu each", count, size);
    
    lldesc_t* dma = (lldesc_t*)heap_caps_malloc(count * sizeof(lldesc_t), MALLOC_CAP_DMA);
    if (!dma) {
        ESP_LOGE(TAG, "Failed to allocate DMA descriptors");
        return NULL;
    }
    
    for (int x = 0; x < count; x++) {
        dma[x].size = size;
        dma[x].length = 0;
        dma[x].sosf = 0;
        dma[x].eof = (x == (count - 1)) ? 1 : 0;  // Set EOF flag on last descriptor
        dma[x].owner = 1;
        dma[x].buf = (buffer + size * x);
        dma[x].empty = (x == (count - 1)) ? 0 : (uint32_t)&dma[x + 1];  // Last points to NULL
    }
    
    return dma;
}

esp_err_t ll_cam_dvp_init(const camera_config_t* config) {
    if (s_cam_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    esp_err_t ret;
    cam_obj_t* cam = &s_cam_obj;
    memset(cam, 0, sizeof(cam_obj_t));
    
    // Create semaphore for frame ready
    s_frame_ready = xSemaphoreCreateBinary();
    if (!s_frame_ready) {
        ESP_LOGE(TAG, "Failed to create frame semaphore");
        return ESP_ERR_NO_MEM;
    }
    
    // Configure basic camera parameters
    cam->width = config->frame_size == FRAMESIZE_96X96 ? 96 : 160;
    cam->height = config->frame_size == FRAMESIZE_96X96 ? 96 : 120;
    cam->jpeg_mode = false;  // Always RGB/RGB565 mode
    cam->fb_bytes_per_pixel = 2;  // We output RGB565 (2 bytes per pixel)
    cam->in_bytes_per_pixel = 2;  // Sensor provides 2 bytes per pixel in RGB565
    cam->vsync_pin = config->vsync_pin;  // Store for later use
    
    // Calculate DMA parameters for I2S camera mode
    // For 8-bit I2S sampling with dma_elem_t packing, only 2 bytes in each 4-byte word are valid.
    // Therefore, to capture N camera bytes we need 2N bytes in the DMA buffer.
    cam->frame_camera_bytes = cam->width * cam->height * cam->fb_bytes_per_pixel; // RGB565 bytes
    size_t dma_bytes_needed_for_frame = cam->frame_camera_bytes * 2; // due to packing
    cam->dma_node_buffer_size = 2048;  // Standard node size
    cam->dma_node_cnt = (dma_bytes_needed_for_frame + cam->dma_node_buffer_size - 1) / cam->dma_node_buffer_size;
    cam->dma_half_buffer_size = cam->dma_node_cnt * cam->dma_node_buffer_size; // one half holds one frame
    cam->dma_buffer_size = cam->dma_half_buffer_size * 2;  // Double buffer (ping-pong)
    
    // Allocate DMA buffer
    cam->dma_buffer = (uint8_t*)heap_caps_malloc(cam->dma_buffer_size, MALLOC_CAP_DMA);
    if (!cam->dma_buffer) {
        ESP_LOGE(TAG, "Failed to allocate DMA buffer");
        vSemaphoreDelete(s_frame_ready);
        return ESP_ERR_NO_MEM;
    }
    
    // Allocate DMA descriptors
    cam->dma = allocate_dma_descriptors(cam->dma_node_cnt, cam->dma_node_buffer_size, cam->dma_buffer);
    if (!cam->dma) {
        ESP_LOGE(TAG, "Failed to allocate DMA descriptors");
        free(cam->dma_buffer);
        vSemaphoreDelete(s_frame_ready);
        return ESP_ERR_NO_MEM;
    }
    
    // Configure I2S for camera
    I2S0.conf.rx_reset = 1;
    I2S0.conf.rx_reset = 0;
    I2S0.conf.rx_fifo_reset = 1;
    I2S0.conf.rx_fifo_reset = 0;
    
    I2S0.conf.rx_slave_mod = 1;
    I2S0.conf.rx_right_first = 0;
    I2S0.conf.rx_msb_right = 0;
    I2S0.conf.rx_msb_shift = 0;
    I2S0.conf.rx_mono = 0;
    I2S0.conf.rx_short_sync = 0;
    
    I2S0.conf2.lcd_en = 1;
    I2S0.conf2.camera_en = 1;
    
    I2S0.sample_rate_conf.rx_bits_mod = 8;
    I2S0.fifo_conf.rx_fifo_mod = SM_0A0B_0C0D;  // 8-bit camera sampling mode
    I2S0.fifo_conf.rx_fifo_mod_force_en = 1;
    I2S0.conf_chan.rx_chan_mod = 1;
    
    // Configure GPIO matrix to connect camera pins to I2S
    ret = configure_camera_gpio(config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure camera GPIO: %s", esp_err_to_name(ret));
        free(cam->dma);
        free(cam->dma_buffer);
        vSemaphoreDelete(s_frame_ready);
        return ret;
    }
    
    // Configure I2S DMA
    I2S0.lc_conf.ahbm_rst = 1;
    I2S0.lc_conf.ahbm_rst = 0;
    I2S0.rx_eof_num = cam->dma_half_buffer_size / sizeof(dma_elem_t);
    I2S0.in_link.addr = ((uint32_t)&cam->dma[0]) & 0xfffff;
    
    // Install I2S DMA interrupt
    ret = esp_intr_alloc(ETS_I2S0_INTR_SOURCE, ESP_INTR_FLAG_LEVEL1 | ESP_INTR_FLAG_IRAM,
                                   ll_cam_dma_isr, cam, &s_i2s_intr_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install I2S interrupt: %s", esp_err_to_name(ret));
        free(cam->dma);
        free(cam->dma_buffer);
        vSemaphoreDelete(s_frame_ready);
        return ret;
    }
    
    // Install VSYNC GPIO interrupt (handle already-installed case)
    ret = gpio_install_isr_service(ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_IRAM);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to install GPIO ISR service: %s", esp_err_to_name(ret));
        free(cam->dma);
        free(cam->dma_buffer);
        vSemaphoreDelete(s_frame_ready);
        return ret;
    }
    gpio_isr_handler_add(config->vsync_pin, ll_cam_vsync_gpio_isr, NULL);
    gpio_intr_disable(config->vsync_pin);  // Start disabled
    
    // Enable I2S interrupts
    I2S_ISR_ENABLE(in_suc_eof);
    
    s_cam_initialized = true;
    ESP_LOGI(TAG, "Camera DVP initialized: %dx%d", cam->width, cam->height);
    return ESP_OK;
}

esp_err_t ll_cam_dvp_capture(uint8_t* rgb_buffer, uint32_t timeout_ms) {
    if (!s_cam_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    cam_obj_t* cam = &s_cam_obj;
    
    // Clear any pending semaphores and flags
    xSemaphoreTake(s_frame_ready, 0);
    s_vsync_detected = false;
    
    ESP_LOGD(TAG, "Starting VSYNC-synchronized DMA capture...");
    
    // Reset DMA system
    I2S0.lc_conf.in_rst = 1;
    I2S0.lc_conf.in_rst = 0;
    I2S0.lc_conf.ahbm_rst = 1; 
    I2S0.lc_conf.ahbm_rst = 0;
    
    // Configure DMA link
    I2S0.in_link.addr = ((uint32_t)&cam->dma[0]) & 0xfffff;
    
    // Enable VSYNC interrupt to detect frame start
    gpio_intr_enable(cam->vsync_pin);
    
    // Check VSYNC pin state before waiting
    int vsync_level = gpio_get_level(cam->vsync_pin);
    ESP_LOGD(TAG, "VSYNC pin %d initial level: %d", cam->vsync_pin, vsync_level);
    
    // Wait for VSYNC edge (frame start) - this ensures proper frame sync
    uint32_t vsync_start = esp_timer_get_time() / 1000; // Convert to milliseconds
    uint32_t check_count = 0;
    while (!s_vsync_detected && (esp_timer_get_time() / 1000 - vsync_start) < timeout_ms) {
        // Check pin level periodically for debugging
        if (check_count % 100 == 0) {
            int current_level = gpio_get_level(cam->vsync_pin);
            ESP_LOGD(TAG, "VSYNC check %u: pin level = %d", check_count, current_level);
        }
        check_count++;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    if (!s_vsync_detected) {
        gpio_intr_disable(cam->vsync_pin);
        int final_level = gpio_get_level(cam->vsync_pin);
        ESP_LOGE(TAG, "VSYNC timeout - no frame start detected. Final pin level: %d", final_level);
        return ESP_ERR_TIMEOUT;
    }
    
    ESP_LOGD(TAG, "VSYNC detected, starting DMA capture");
    
    // Now start DMA capture synchronized with frame
    I2S0.in_link.start = 1;
    I2S0.conf.rx_start = 1;
    
    // Wait for DMA frame to be ready
    if (xSemaphoreTake(s_frame_ready, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        I2S0.conf.rx_start = 0;
        gpio_intr_disable(cam->vsync_pin);
        ESP_LOGE(TAG, "DMA frame capture timeout after VSYNC");
        return ESP_ERR_TIMEOUT;
    }
    
    // Stop I2S reception and disable VSYNC interrupt
    I2S0.conf.rx_start = 0;
    gpio_intr_disable(cam->vsync_pin);
    
    // Copy and filter DMA data to RGB565 buffer
    size_t rgb_len = ll_cam_dma_filter_rgb(rgb_buffer, cam->dma_buffer, cam->dma_half_buffer_size);
    if (rgb_len > cam->frame_camera_bytes) {
        rgb_len = cam->frame_camera_bytes;
    }
    
    ESP_LOGD(TAG, "Frame captured: %zu bytes RGB565 data", rgb_len);
    return ESP_OK;
}

esp_err_t ll_cam_dvp_capture_no_vsync(uint8_t* rgb_buffer, uint32_t timeout_ms) {
    if (!s_cam_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    cam_obj_t* cam = &s_cam_obj;
    
    // Clear any pending semaphores
    xSemaphoreTake(s_frame_ready, 0);
    
    ESP_LOGD(TAG, "Starting single-shot DMA capture (no VSYNC wait)");
    
    // Reset DMA system
    I2S0.lc_conf.in_rst = 1;
    I2S0.lc_conf.in_rst = 0;
    I2S0.lc_conf.ahbm_rst = 1; 
    I2S0.lc_conf.ahbm_rst = 0;
    
    // Configure DMA link
    I2S0.in_link.addr = ((uint32_t)&cam->dma[0]) & 0xfffff;
    
    // Start DMA capture immediately
    I2S0.in_link.start = 1;
    I2S0.conf.rx_start = 1;
    
    ESP_LOGD(TAG, "DMA started, waiting for data...");
    
    // Wait for DMA frame to be ready
    if (xSemaphoreTake(s_frame_ready, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        I2S0.conf.rx_start = 0;
        ESP_LOGE(TAG, "DMA capture timeout - no data received");
        return ESP_ERR_TIMEOUT;
    }
    
    // Stop I2S reception
    I2S0.conf.rx_start = 0;
    
    // Copy and filter DMA data to RGB565 buffer
    size_t rgb_len = ll_cam_dma_filter_rgb(rgb_buffer, cam->dma_buffer, cam->dma_half_buffer_size);
    if (rgb_len > cam->frame_camera_bytes) {
        rgb_len = cam->frame_camera_bytes;
    }
    
    ESP_LOGD(TAG, "Single-shot frame captured: %zu bytes RGB565 data", rgb_len);
    return ESP_OK;
}

void ll_cam_dvp_deinit() {
    if (!s_cam_initialized) {
        return;
    }
    
    // Disable interrupts
    if (s_i2s_intr_handle) {
        esp_intr_free(s_i2s_intr_handle);
        s_i2s_intr_handle = NULL;
    }
    
    // Remove VSYNC GPIO interrupt
    if (s_cam_obj.vsync_pin >= 0) {
        gpio_isr_handler_remove(s_cam_obj.vsync_pin);
        gpio_intr_disable(s_cam_obj.vsync_pin);
    }
    
    // Free resources
    if (s_cam_obj.dma) {
        free(s_cam_obj.dma);
        s_cam_obj.dma = NULL;
    }
    
    if (s_cam_obj.dma_buffer) {
        free(s_cam_obj.dma_buffer);
        s_cam_obj.dma_buffer = NULL;
    }
    
    if (s_frame_ready) {
        vSemaphoreDelete(s_frame_ready);
        s_frame_ready = NULL;
    }
    
    s_cam_initialized = false;
    ESP_LOGI(TAG, "Camera DVP deinitialized");
}

esp_err_t ll_cam_dvp_capture_with_trigger(uint8_t* rgb_buffer, uint32_t timeout_ms, esp_err_t (*trigger_fn)(void))
{
    if (!s_cam_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    cam_obj_t* cam = &s_cam_obj;

    // Clear any pending semaphores
    xSemaphoreTake(s_frame_ready, 0);

    ESP_LOGD(TAG, "Starting DMA, then triggering snapshot...");

    // Reset DMA system
    I2S0.lc_conf.in_rst = 1;
    I2S0.lc_conf.in_rst = 0;
    I2S0.lc_conf.ahbm_rst = 1;
    I2S0.lc_conf.ahbm_rst = 0;

    // Configure DMA link
    I2S0.in_link.addr = ((uint32_t)&cam->dma[0]) & 0xfffff;

    // Start DMA capture immediately
    I2S0.in_link.start = 1;
    I2S0.conf.rx_start = 1;

    // Trigger
    if (trigger_fn) {
        esp_err_t tr = trigger_fn();
        if (tr != ESP_OK) {
            I2S0.conf.rx_start = 0;
            ESP_LOGE(TAG, "Trigger callback failed: %s", esp_err_to_name(tr));
            return tr;
        }
    }

    // Wait for DMA frame to be ready
    if (xSemaphoreTake(s_frame_ready, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        I2S0.conf.rx_start = 0;
        ESP_LOGE(TAG, "DMA capture timeout after trigger");
        return ESP_ERR_TIMEOUT;
    }

    // Stop I2S reception
    I2S0.conf.rx_start = 0;

    // Copy and filter DMA data to RGB565 buffer
    size_t rgb_len = ll_cam_dma_filter_rgb(rgb_buffer, cam->dma_buffer, cam->dma_half_buffer_size);
    if (rgb_len > cam->frame_camera_bytes) {
        rgb_len = cam->frame_camera_bytes;
    }

    ESP_LOGD(TAG, "Triggered frame captured: %zu bytes RGB565 data", rgb_len);
    return ESP_OK;
}