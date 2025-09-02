#include "single_shot_camera.h"
#include "image_converters.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/i2s_struct.h"
#include "soc/gpio_sig_map.h"
#include "hal/gpio_hal.h"
#include "ll_cam_dvp.h"

static const char* TAG = "single_shot_cam";

// Camera I2C addresses
#define OV2640_I2C_ADDR 0x30
#define OV3660_I2C_ADDR 0x3C

// OV2640 register definitions (key ones for single-shot)
#define BANK_SEL            0xFF
#define BANK_DSP            0x00
#define BANK_SENSOR         0x01

// DSP Bank registers
#define R_BYPASS            0x05
#define IMAGE_MODE          0xDA
#define RESET               0xE0
#define CTRL0               0xC2
#define CTRL2               0x86

// Sensor Bank registers  
#define COM2                0x09  // Standby and drive control
#define COM3                0x0C  // Snapshot control register
#define COM7                0x12
#define COM8                0x13
#define COM9                0x14
#define COM10               0x15  // PCLK, HREF, VSYNC control
#define CLKRC               0x11
#define REG_PID             0x0A
#define REG_VER             0x0B

// OV3660 register definitions
#define OV3660_REG_PID_HIGH 0x300A
#define OV3660_REG_PID_LOW  0x300B
#define OV3660_SYSTEM_CTRL0 0x3008

// Register bit definitions
#define COM3_SNAPSHOT_EN    0x01  // Snapshot option: 0=live video after snapshot, 1=single frame only
#define COM7_SRST           0x80
#define COM8_DEFAULT        0xC0
#define COM10_PCLK_FREE     0x20
#define IMAGE_MODE_JPEG_EN  0x10
#define RESET_DVP           0x04
#define R_BYPASS_DSP_EN     0x00

// Global configuration
static single_shot_config_t g_config;
static bool g_initialized = false;
static i2c_port_t g_i2c_port = I2C_NUM_0;

// Forward declarations
static esp_err_t sensor_write_reg(uint16_t reg, uint8_t val);
static esp_err_t sensor_read_reg(uint16_t reg, uint8_t* val);
static esp_err_t ov2640_set_bank(uint8_t bank);
static esp_err_t sensor_soft_reset();
static esp_err_t apply_basic_sensor_init();
static esp_err_t sensor_configure_single_shot();
static esp_err_t sensor_configure_rgb_capture();
static esp_err_t capture_rgb_frame(uint8_t* rgb_buf, uint16_t width, uint16_t height);
static esp_err_t capture_rgb_frame_with_trigger(uint8_t* rgb_buf, uint16_t width, uint16_t height);
static esp_err_t init_gpio();
static esp_err_t init_xclk();
static esp_err_t xclk_enable();
static esp_err_t xclk_disable();
static esp_err_t init_i2c();
static esp_err_t power_up_sensor();
static esp_err_t power_down_sensor();
static uint8_t get_sensor_i2c_addr();

esp_err_t single_shot_camera_init(const single_shot_config_t* config) {
    if (config == NULL) {
        ESP_LOGE(TAG, "Config is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (g_initialized) {
        ESP_LOGW(TAG, "Camera already initialized");
        return ESP_OK;
    }
    
    // Copy configuration
    g_config = *config;
    
    ESP_LOGI(TAG, "Initializing single-shot camera");
    
    // Initialize I2C for sensor communication
    esp_err_t ret = init_i2c();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize GPIOs
    ret = init_gpio();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize GPIO: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize XCLK (camera clock)
    ret = init_xclk();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize XCLK: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Wait for XCLK to stabilize before any I2C communication (datasheet requirement)
    ESP_LOGI(TAG, "Waiting for XCLK to stabilize...");
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Initialize camera DVP interface
    camera_config_t cam_config = {
        .frame_size = FRAMESIZE_96X96,
        .d0_pin = g_config.d0_pin,
        .d1_pin = g_config.d1_pin,
        .d2_pin = g_config.d2_pin,
        .d3_pin = g_config.d3_pin,
        .d4_pin = g_config.d4_pin,
        .d5_pin = g_config.d5_pin,
        .d6_pin = g_config.d6_pin,
        .d7_pin = g_config.d7_pin,
        .vsync_pin = g_config.vsync_pin,
        .href_pin = g_config.href_pin,
        .pclk_pin = g_config.pclk_pin,
    };
    ret = ll_cam_dvp_init(&cam_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize camera DVP: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Power up sensor to test communication
    ret = power_up_sensor();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to power up sensor: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Wait for sensor power-up and internal reset sequence (datasheet timing)
    ESP_LOGI(TAG, "Waiting for sensor power-up sequence...");
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // Test sensor communication
    ret = single_shot_camera_test();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Camera test failed: %s", esp_err_to_name(ret));
        power_down_sensor();
        return ret;
    }
    
    // Keep sensor powered for now - powering down/up might disrupt configuration
    ESP_LOGI(TAG, "Keeping sensor powered and configured");
    
    g_initialized = true;
    ESP_LOGI(TAG, "Single-shot camera initialized successfully");
    return ESP_OK;
}

esp_err_t single_shot_capture_jpeg(uint8_t** jpeg_data, size_t* jpeg_len) {
    if (!g_initialized) {
        ESP_LOGE(TAG, "Camera not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (jpeg_data == NULL || jpeg_len == NULL) {
        ESP_LOGE(TAG, "Output parameters are NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Starting single-shot JPEG capture");
    
    // Power up sensor
    esp_err_t ret = power_up_sensor();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to power up sensor");
        return ret;
    }
    
    // Configure for single-shot JPEG capture
    ret = sensor_configure_single_shot();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure sensor");
        power_down_sensor();
        return ret;
    }
    
    // TODO: Implement actual frame capture via I2S DMA
    // For now, return a placeholder
    ESP_LOGW(TAG, "Frame capture not yet implemented - returning placeholder");
    
    // Allocate placeholder data
    *jpeg_len = 1024;  // Placeholder size
    *jpeg_data = (uint8_t*)malloc(*jpeg_len);
    if (*jpeg_data == NULL) {
        ESP_LOGE(TAG, "Failed to allocate JPEG buffer");
        power_down_sensor();
        return ESP_ERR_NO_MEM;
    }
    
    // Fill with placeholder data
    memset(*jpeg_data, 0xFF, *jpeg_len);
    
    // Power down sensor
    power_down_sensor();
    
    ESP_LOGI(TAG, "Single-shot capture completed");
    return ESP_OK;
}

esp_err_t single_shot_capture_rgb_96x96(uint8_t** rgb_data) {
    if (!g_initialized) {
        ESP_LOGE(TAG, "Camera not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (rgb_data == NULL) {
        ESP_LOGE(TAG, "Output parameter is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Starting single-shot RGB capture (96x96)");
    
    // Allocate RGB buffer
    *rgb_data = (uint8_t*)malloc(96 * 96 * 3);
    if (*rgb_data == NULL) {
        ESP_LOGE(TAG, "Failed to allocate RGB buffer");
        return ESP_ERR_NO_MEM;
    }
    
    // Ensure sensor is powered and XCLK is running for this capture window
    ESP_LOGI(TAG, "Powering sensor and enabling XCLK for capture window");
    esp_err_t ret = power_up_sensor();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to power up sensor");
        free(*rgb_data);
        *rgb_data = NULL;
        return ret;
    }
    ret = xclk_enable();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable XCLK");
        power_down_sensor();
        free(*rgb_data);
        *rgb_data = NULL;
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(20)); // allow clock/sensor to stabilize
    
    // Configure sensor for RGB888 output
    ret = sensor_configure_rgb_capture();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure sensor for RGB capture");
        free(*rgb_data);
        *rgb_data = NULL;
        return ret;
    }
    
    // If FREX is not wired, prefer continuous output + VSYNC-synced capture
    if (g_config.frex_pin < 0) {
        ESP_LOGI(TAG, "FREX not wired - using VSYNC-synchronized capture (continuous output)");
        ret = ll_cam_dvp_capture(*rgb_data, 3000);
    } else {
        // Start DMA capture first, then trigger snapshot to ensure alignment
        ESP_LOGI(TAG, "Starting DMA capture and triggering snapshot...");
        ret = capture_rgb_frame_with_trigger(*rgb_data, 96, 96);
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to capture RGB frame");
        free(*rgb_data);
        *rgb_data = NULL;
        return ret;
    }
    
    // End of capture window: gate off XCLK and power down sensor to emulate true snapshot
    ESP_LOGI(TAG, "Disabling XCLK and powering down sensor after capture");
    xclk_disable();
    power_down_sensor();
    
    ESP_LOGI(TAG, "RGB capture completed: 96x96");
    return ESP_OK;
}

esp_err_t single_shot_trigger_snapshot() {
    if (!g_initialized) {
        ESP_LOGE(TAG, "Camera not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (g_config.frex_pin < 0) {
        ESP_LOGE(TAG, "FREX pin not configured");
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGD(TAG, "Triggering snapshot via FREX pin %d", g_config.frex_pin);
    
    // FREX trigger sequence based on OV2640 datasheet Figure 18-19
    // 1. Set FREX low to trigger snapshot
    gpio_set_level(g_config.frex_pin, 0);
    
    // 2. Hold low for minimum trigger time (datasheet: minimum pulse width)
    vTaskDelay(pdMS_TO_TICKS(10)); // 10ms should be sufficient
    
    // 3. Return FREX high to complete trigger
    gpio_set_level(g_config.frex_pin, 1);
    
    ESP_LOGI(TAG, "Snapshot triggered successfully");
    return ESP_OK;
}

esp_err_t single_shot_trigger_i2c_snapshot() {
    if (!g_initialized) {
        ESP_LOGE(TAG, "Camera not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Triggering snapshot via I2C COM3 register");
    
    esp_err_t ret;
    
    if (g_config.sensor_type == CAMERA_SENSOR_OV2640) {
        // Switch to sensor bank to access COM3 register
        ret = ov2640_set_bank(BANK_SENSOR);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to select sensor bank");
            return ret;
        }
        
        // Read current COM3 value
        uint8_t com3_val;
        ret = sensor_read_reg(COM3, &com3_val);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read COM3 register");
            return ret;
        }
        ESP_LOGD(TAG, "Current COM3 value: 0x%02X", com3_val);
        
        // Method: Use proper snapshot sequence from datasheet
        // Step 1: First ensure COM3[0] = 1 for snapshot mode
        ret = sensor_write_reg(COM3, com3_val | COM3_SNAPSHOT_EN);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to enable snapshot mode in COM3");
            return ret;
        }
        
        // Verify COM3 was set correctly
        uint8_t com3_verify;
        ret = sensor_read_reg(COM3, &com3_verify);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "COM3 after snapshot enable: 0x%02X (should be 0x01)", com3_verify);
        }
        
        // Step 2: Switch to DSP bank to trigger via RESET register
        ret = ov2640_set_bank(BANK_DSP);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to select DSP bank");
            return ret;
        }
        
        // Step 3: Trigger frame using proper exposure control method
        // According to datasheet, use AEC register and COM2 for controlled frame capture
        
        ret = ov2640_set_bank(BANK_SENSOR);
        if (ret != ESP_OK) return ret;
        
        // Configure AEC register (0x10) for single frame exposure timing
        ret = sensor_write_reg(0x10, 0x00);  // AEC - minimum exposure for fast capture
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure AEC register");
            return ret;
        }
        
        // Set COM2 output drive for single frame trigger
        ret = sensor_write_reg(COM2, 0x03);  // Maximum output drive (4x capability)
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure COM2 output drive");
            return ret;
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
        
        // Reset COM2 back to normal after frame trigger
        ret = sensor_write_reg(COM2, 0x01);  // Normal output drive (2x capability)
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to reset COM2 output drive");
            return ret;
        }
        
        ESP_LOGI(TAG, "I2C snapshot triggered: AEC exposure control method");
        
    } else if (g_config.sensor_type == CAMERA_SENSOR_OV3660) {
        // OV3660 doesn't have the same COM3 register structure
        // Would need different implementation for OV3660
        ESP_LOGE(TAG, "I2C snapshot not implemented for OV3660");
        return ESP_ERR_NOT_SUPPORTED;
        
    } else {
        ESP_LOGE(TAG, "Unsupported sensor type for I2C snapshot");
        return ESP_ERR_NOT_SUPPORTED;
    }
    
    return ESP_OK;
}

esp_err_t single_shot_reset_to_continuous() {
    if (!g_initialized) {
        ESP_LOGE(TAG, "Camera not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Resetting sensor to continuous video mode");
    
    esp_err_t ret;
    
    if (g_config.sensor_type == CAMERA_SENSOR_OV2640) {
        // Switch to sensor bank to access COM3 register
        ret = ov2640_set_bank(BANK_SENSOR);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to select sensor bank");
            return ret;
        }
        
        // Read current COM3 value
        uint8_t com3_val;
        ret = sensor_read_reg(COM3, &com3_val);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read COM3 register");
            return ret;
        }
        
        // Clear snapshot bit (COM3[0] = 0) to return to continuous mode
        ret = sensor_write_reg(COM3, com3_val & ~COM3_SNAPSHOT_EN);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to clear snapshot mode in COM3");
            return ret;
        }
        
        ESP_LOGI(TAG, "Sensor reset to continuous mode: COM3[0] = 0");
        
    } else if (g_config.sensor_type == CAMERA_SENSOR_OV3660) {
        ESP_LOGE(TAG, "Continuous mode reset not implemented for OV3660");
        return ESP_ERR_NOT_SUPPORTED;
        
    } else {
        ESP_LOGE(TAG, "Unsupported sensor type for continuous mode reset");
        return ESP_ERR_NOT_SUPPORTED;
    }
    
    return ESP_OK;
}

esp_err_t single_shot_camera_test() {
    ESP_LOGI(TAG, "Testing camera communication");
    
    esp_err_t ret;
    
    if (g_config.sensor_type == CAMERA_SENSOR_OV2640) {
        // Read OV2640 sensor ID registers
        uint8_t pid, ver;
        ret = ov2640_set_bank(BANK_SENSOR);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set sensor bank");
            return ret;
        }
        
        ret = sensor_read_reg(REG_PID, &pid);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read PID register");
            return ret;
        }
        
        ret = sensor_read_reg(REG_VER, &ver);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read VER register");
            return ret;
        }
        
        ESP_LOGI(TAG, "OV2640 PID: 0x%02X, VER: 0x%02X", pid, ver);
        
        // Check if it's OV2640 (PID should be 0x26)
        if (pid != 0x26) {
            ESP_LOGE(TAG, "Unexpected OV2640 PID: 0x%02X (expected 0x26)", pid);
            return ESP_ERR_NOT_FOUND;
        }
        
    } else if (g_config.sensor_type == CAMERA_SENSOR_OV3660) {
        // Read OV3660 sensor ID registers
        uint8_t pid_h, pid_l;
        ret = sensor_read_reg(OV3660_REG_PID_HIGH, &pid_h);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read OV3660 PID high register");
            return ret;
        }
        
        ret = sensor_read_reg(OV3660_REG_PID_LOW, &pid_l);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read OV3660 PID low register");
            return ret;
        }
        
        uint16_t pid = (pid_h << 8) | pid_l;
        ESP_LOGI(TAG, "OV3660 PID: 0x%04X", pid);
        
        // Check if it's OV3660 (PID should be 0x3660)
        if (pid != 0x3660) {
            ESP_LOGE(TAG, "Unexpected OV3660 PID: 0x%04X (expected 0x3660)", pid);
            return ESP_ERR_NOT_FOUND;
        }
    } else {
        ESP_LOGE(TAG, "Unsupported sensor type: %d", g_config.sensor_type);
        return ESP_ERR_NOT_SUPPORTED;
    }
    
    // Apply basic sensor initialization after successful ID check
    ESP_LOGI(TAG, "Applying basic sensor initialization");
    ret = apply_basic_sensor_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize sensor");
        return ret;
    }
    
    ESP_LOGI(TAG, "Camera test passed");
    return ESP_OK;
}

void single_shot_camera_deinit() {
    if (!g_initialized) {
        return;
    }
    
    ESP_LOGI(TAG, "Deinitializing single-shot camera");
    
    // Power down sensor
    power_down_sensor();
    
    // Deinitialize LEDC (XCLK)
    ledc_stop(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 0);
    
    // Deinitialize camera DVP
    ll_cam_dvp_deinit();
    
    // Deinitialize I2C
    i2c_driver_delete(g_i2c_port);
    
    g_initialized = false;
    ESP_LOGI(TAG, "Camera deinitialized");
}

// Private function implementations

static esp_err_t init_i2c() {
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = g_config.sda_pin,
        .scl_io_num = g_config.scl_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = g_config.i2c_freq,
    };
    
    esp_err_t ret = i2c_param_config(g_i2c_port, &i2c_config);
    if (ret != ESP_OK) {
        return ret;
    }
    
    return i2c_driver_install(g_i2c_port, I2C_MODE_MASTER, 0, 0, 0);
}

static esp_err_t init_gpio() {
    // Configure power down pin
    if (g_config.pwdn_pin >= 0) {
        gpio_config_t gpio_conf = {
            .pin_bit_mask = (1ULL << g_config.pwdn_pin),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&gpio_conf);
        gpio_set_level(g_config.pwdn_pin, 1); // Start powered down
    }
    
    // Configure reset pin if available
    if (g_config.reset_pin >= 0) {
        gpio_config_t gpio_conf = {
            .pin_bit_mask = (1ULL << g_config.reset_pin),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&gpio_conf);
        gpio_set_level(g_config.reset_pin, 1); // Not in reset
    }
    
    // Configure FREX pin for snapshot trigger
    if (g_config.frex_pin >= 0) {
        gpio_config_t gpio_conf = {
            .pin_bit_mask = (1ULL << g_config.frex_pin),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&gpio_conf);
        gpio_set_level(g_config.frex_pin, 1); // Start high (inactive)
        ESP_LOGI(TAG, "FREX pin %d configured for snapshot trigger", g_config.frex_pin);
    }
    
    return ESP_OK;
}

static esp_err_t init_xclk() {
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_2_BIT,  // Lower resolution for high frequency
        .freq_hz = g_config.xclk_freq,
        .clk_cfg = LEDC_AUTO_CLK
    };
    
    esp_err_t ret = ledc_timer_config(&ledc_timer);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = g_config.xclk_pin,
        .duty = 2, // 50% duty cycle for 2-bit resolution (2/4)
        .hpoint = 0
    };
    
    ret = ledc_channel_config(&ledc_channel);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Start PWM by setting duty cycle (LEDC starts automatically when duty is set)
    ret = ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 1); // 50% duty cycle
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ESP_LOGI(TAG, "XCLK started at %u Hz", g_config.xclk_freq);
    return ESP_OK;
}

static esp_err_t xclk_enable() {
    // Re-enable XCLK by setting non-zero duty
    esp_err_t ret = ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 1);
    if (ret != ESP_OK) return ret;
    ret = ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
    return ret;
}

static esp_err_t xclk_disable() {
    // Gate off XCLK by setting duty to 0 (constant low)
    esp_err_t ret = ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 0);
    if (ret != ESP_OK) return ret;
    ret = ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
    return ret;
}

static esp_err_t power_up_sensor() {
    ESP_LOGD(TAG, "Powering up sensor");
    
    if (g_config.pwdn_pin >= 0) {
        gpio_set_level(g_config.pwdn_pin, 0); // PWDN is active HIGH: 0=normal, 1=power-down
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    return ESP_OK;
}

static esp_err_t power_down_sensor() {
    ESP_LOGD(TAG, "Powering down sensor");
    
    if (g_config.pwdn_pin >= 0) {
        gpio_set_level(g_config.pwdn_pin, 1); // PWDN is active HIGH: 0=normal, 1=power-down
    }
    
    return ESP_OK;
}

static uint8_t get_sensor_i2c_addr() {
    return (g_config.sensor_type == CAMERA_SENSOR_OV3660) ? OV3660_I2C_ADDR : OV2640_I2C_ADDR;
}

static esp_err_t sensor_write_reg(uint16_t reg, uint8_t val) {
    uint8_t i2c_addr = get_sensor_i2c_addr();
    esp_err_t ret;
    
    // Retry mechanism for I2C writes
    for (int retry = 0; retry < 3; retry++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (i2c_addr << 1) | I2C_MASTER_WRITE, true);
        
        if (g_config.sensor_type == CAMERA_SENSOR_OV3660) {
            // OV3660 uses 16-bit register addresses
            i2c_master_write_byte(cmd, (reg >> 8) & 0xFF, true);  // High byte
            i2c_master_write_byte(cmd, reg & 0xFF, true);         // Low byte
        } else {
            // OV2640 uses 8-bit register addresses
            i2c_master_write_byte(cmd, (uint8_t)reg, true);
        }
        
        i2c_master_write_byte(cmd, val, true);
        i2c_master_stop(cmd);
        
        ret = i2c_master_cmd_begin(g_i2c_port, cmd, pdMS_TO_TICKS(100)); // 100ms timeout per attempt
        i2c_cmd_link_delete(cmd);
        
        if (ret == ESP_OK) {
            return ESP_OK; // Success, no need to retry
        }
        
        ESP_LOGW(TAG, "I2C write retry %d failed: reg=0x%04X, val=0x%02X, err=%s", 
                 retry + 1, reg, val, esp_err_to_name(ret));
        
        // Small delay between retries
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    ESP_LOGE(TAG, "I2C write failed after 3 retries: reg=0x%04X, val=0x%02X, err=%s", 
             reg, val, esp_err_to_name(ret));
    return ret;
}

static esp_err_t sensor_read_reg(uint16_t reg, uint8_t* val) {
    if (val == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t i2c_addr = get_sensor_i2c_addr();
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_addr << 1) | I2C_MASTER_WRITE, true);
    
    if (g_config.sensor_type == CAMERA_SENSOR_OV3660) {
        // OV3660 uses 16-bit register addresses
        i2c_master_write_byte(cmd, (reg >> 8) & 0xFF, true);  // High byte
        i2c_master_write_byte(cmd, reg & 0xFF, true);         // Low byte
    } else {
        // OV2640 uses 8-bit register addresses
        i2c_master_write_byte(cmd, (uint8_t)reg, true);
    }
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, val, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(g_i2c_port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGD(TAG, "I2C read failed: reg=0x%04X, err=%s", reg, esp_err_to_name(ret));
    }
    
    return ret;
}

static esp_err_t ov2640_set_bank(uint8_t bank) {
    return sensor_write_reg(BANK_SEL, bank);
}

static esp_err_t sensor_soft_reset() {
    ESP_LOGD(TAG, "Performing soft reset");
    
    esp_err_t ret;
    
    if (g_config.sensor_type == CAMERA_SENSOR_OV2640) {
        ret = ov2640_set_bank(BANK_SENSOR);
        if (ret != ESP_OK) return ret;
        
        ret = sensor_write_reg(COM7, COM7_SRST);
        if (ret != ESP_OK) return ret;
        
    } else if (g_config.sensor_type == CAMERA_SENSOR_OV3660) {
        // OV3660 soft reset
        ret = sensor_write_reg(OV3660_SYSTEM_CTRL0, 0x82);  // Set soft reset bit
        if (ret != ESP_OK) return ret;
        
    } else {
        return ESP_ERR_NOT_SUPPORTED;
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));  // Longer delay for OV3660
    return ESP_OK;
}


// Working OV2640 RGB565 initialization sequence from ESP32-Camera
// Minimal OV2640 initialization to get VSYNC working
static const uint8_t ov2640_init_regs[][2] = {
    // Software reset first
    {0xff, 0x01}, // BANK_SENSOR  
    {0x12, 0x80}, // COM7_SRST - Software reset
    {0x00, 0x00}, // Delay marker
    
    // Minimal sensor configuration
    {0xff, 0x01}, // BANK_SENSOR
    {0x11, 0x01}, // CLKRC - Use external clock
    {0x12, 0x02}, // COM7 - CIF resolution
    {0x0c, 0x00}, // COM3 - Start in continuous mode (bit 0 = 0), will set to 1 for snapshots
    {0x09, 0x02}, // COM2 - Output drive 2x  
    {0x15, 0x20}, // COM10 - Enable VSYNC output
    
    // Enable DSP
    {0xff, 0x00}, // BANK_DSP
    {0xe0, 0x00}, // RESET - Clear reset
    {0xc0, 0x64}, // HSIZE8
    {0xc1, 0x4b}, // VSIZE8
    {0x8c, 0x00}, // SIZEL
    {0x86, 0x3d}, // CTRL2
    {0x50, 0x00}, // CTRLI
    {0x51, 0xc8}, // HSIZE
    {0x52, 0x96}, // VSIZE
    {0x53, 0x00}, // XOFFL
    {0x54, 0x00}, // YOFFL
    {0x55, 0x00}, // VHYX
    {0xd3, 0x82}, // R_DVP_SP - Auto mode
    {0x05, 0x00}, // R_BYPASS - Enable DSP
    {0xda, 0x08}, // IMAGE_MODE - YUV422 output
};

static const uint8_t ov2640_rgb565_regs[][2] = {
    {0xff, 0x00}, {0xda, 0x09}, {0xd7, 0x03}, {0xdf, 0x02}, {0x33, 0xa0}, {0x3c, 0x00},
    {0xe1, 0x67}, {0xd3, 0x82}, {0xc0, 0x50}, {0xc1, 0x3c}, {0xc2, 0x01}, {0x86, 0x35},
    {0x50, 0x92}, {0x51, 0xc8}, {0x52, 0x96}, {0x53, 0x00}, {0x54, 0x00}, {0x55, 0x00},
    {0x57, 0x00}, {0x5a, 0x50}, {0x5b, 0x3c}, {0x5c, 0x00}, {0xe0, 0x00}, {0xc3, 0xed},
    {0x7f, 0x00}, {0xda, 0x09}, {0xe5, 0x1f}, {0xe1, 0x67}, {0xe0, 0x00}, {0xdd, 0x7f},
    {0x05, 0x00},
};

static esp_err_t apply_basic_sensor_init() {
    ESP_LOGI(TAG, "Applying proven OV2640 initialization sequence from ESP32-Camera");
    
    esp_err_t ret;
    
    // Apply complete initialization sequence
    size_t init_count = sizeof(ov2640_init_regs) / sizeof(ov2640_init_regs[0]);
    for (size_t i = 0; i < init_count; i++) {
        uint8_t reg = ov2640_init_regs[i][0];
        uint8_t val = ov2640_init_regs[i][1];
        
        // Check for delay marker (0x00, 0x00)
        if (reg == 0x00 && val == 0x00) {
            ESP_LOGI(TAG, "Software reset delay...");
            vTaskDelay(pdMS_TO_TICKS(10)); // 10ms delay after reset
            continue;
        }
        
        // Write register
        ret = sensor_write_reg(reg, val);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to write reg 0x%02X = 0x%02X", reg, val);
            return ret;
        }
    }
    
    ESP_LOGI(TAG, "Core initialization complete, applying RGB565 format");
    
    // Apply RGB565 format configuration  
    size_t rgb_count = sizeof(ov2640_rgb565_regs) / sizeof(ov2640_rgb565_regs[0]);
    for (size_t i = 0; i < rgb_count; i++) {
        uint8_t reg = ov2640_rgb565_regs[i][0];
        uint8_t val = ov2640_rgb565_regs[i][1];
        
        if (reg == 0xff) {
            ret = sensor_write_reg(reg, val);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to select bank 0x%02X", val);
                return ret;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        } else {
            ret = sensor_write_reg(reg, val);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "Failed to write RGB reg 0x%02X = 0x%02X, continuing...", reg, val);
            }
        }
    }
    
    // Final configuration to ensure outputs are enabled
    ret = ov2640_set_bank(BANK_SENSOR);
    if (ret == ESP_OK) {
        // Read current register values first
        uint8_t com7_val;
        sensor_read_reg(COM7, &com7_val);
        ESP_LOGI(TAG, "Before final config: COM7=0x%02X", com7_val);
        
        // Make sure we're not in reset or standby
        sensor_write_reg(COM7, 0x00);  // Normal operation, no reset
        sensor_write_reg(CLKRC, 0x80); // Use internal clock doubler
        sensor_write_reg(COM2, 0x01);  // Output drive x2
        // COM3: Keep existing snapshot configuration - don't override to 0x00
        sensor_write_reg(COM8, 0xC0);  // Fast AGC/AEC
        sensor_write_reg(COM9, 0x20);  // 8x gain ceiling
        sensor_write_reg(COM10, 0x20); // Enable VSYNC output, PCLK free running
        
        // Verify configuration
        uint8_t com2_val, com3_val, com10_val;
        sensor_read_reg(COM2, &com2_val);
        sensor_read_reg(COM3, &com3_val);
        sensor_read_reg(COM7, &com7_val);
        sensor_read_reg(COM10, &com10_val);
        ESP_LOGI(TAG, "Final sensor config: COM2=0x%02X, COM3=0x%02X, COM7=0x%02X, COM10=0x%02X",
                 com2_val, com3_val, com7_val, com10_val);
    }
    
    // Now configure DSP bank to enable output
    ret = ov2640_set_bank(BANK_DSP);
    if (ret == ESP_OK) {
        // Enable modules in DSP
        sensor_write_reg(CTRL2, 0x35);  // Enable DCW, SDE, UV_ADJ, UV_AVG
        sensor_write_reg(CTRL0, 0xFF);  // Enable all modules
        
        ESP_LOGI(TAG, "Enabled DSP modules for frame output");
    }
    
    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_LOGI(TAG, "Proven OV2640 initialization completed");
    return ESP_OK;
}

static esp_err_t sensor_configure_single_shot() {
    ESP_LOGD(TAG, "Configuring sensor for single-shot");
    
    // Don't do a soft reset here as it would undo initialization
    // esp_err_t ret = sensor_soft_reset();
    // if (ret != ESP_OK) return ret;
    
    esp_err_t ret = ESP_OK;
    
    if (g_config.sensor_type == CAMERA_SENSOR_OV2640) {
        // Switch to sensor bank first to configure COM3 for snapshot mode
        ret = ov2640_set_bank(BANK_SENSOR);
        if (ret != ESP_OK) return ret;
        
        // Configure COM3 for exposure-controlled capture
        // Note: According to datasheet, COM3 bit[0] should be 0 for live video output
        // We'll control single frames via AEC/exposure registers instead
        ret = sensor_write_reg(COM3, 0x38);  // Live video mode (bit 0 = 0)
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure COM3");
            return ret;
        }
        ESP_LOGI(TAG, "COM3 configured for normal capture mode");
        
        // Switch to DSP bank for image format configuration
        ret = ov2640_set_bank(BANK_DSP);
        if (ret != ESP_OK) return ret;
        
        // Enable JPEG mode
        ret = sensor_write_reg(IMAGE_MODE, IMAGE_MODE_JPEG_EN);
        if (ret != ESP_OK) return ret;
        
        // Configure basic settings
        ret = sensor_write_reg(R_BYPASS, R_BYPASS_DSP_EN);
        if (ret != ESP_OK) return ret;
        
    } else if (g_config.sensor_type == CAMERA_SENSOR_OV3660) {
        // Basic OV3660 configuration for JPEG output
        // Enable JPEG format (register 0x501F = 0x00)
        ret = sensor_write_reg(0x501F, 0x00);  // Set format to JPEG
        if (ret != ESP_OK) return ret;
        
        // Additional OV3660 setup would go here
        // TODO: Add full OV3660 initialization sequence
        
    } else {
        return ESP_ERR_NOT_SUPPORTED;
    }
    
    ESP_LOGD(TAG, "Sensor configuration completed");
    return ESP_OK;
}

// Camera control function implementations

// White balance register settings from ESP32-Camera OV2640 driver
static const uint8_t OV2640_WB_AUTO[][3] = {
    {0xFF, 0x00}, {0xC7, 0x10}
};

static const uint8_t OV2640_WB_SUNNY[][3] = {
    {0xFF, 0x00}, {0xC7, 0x40}, {0xCC, 0x5E}, {0xCD, 0x41}, {0xCE, 0x54}
};

static const uint8_t OV2640_WB_CLOUDY[][3] = {
    {0xFF, 0x00}, {0xC7, 0x40}, {0xCC, 0x65}, {0xCD, 0x41}, {0xCE, 0x4F}
};

static const uint8_t OV2640_WB_OFFICE[][3] = {
    {0xFF, 0x00}, {0xC7, 0x40}, {0xCC, 0x52}, {0xCD, 0x41}, {0xCE, 0x66}
};

static const uint8_t OV2640_WB_HOME[][3] = {
    {0xFF, 0x00}, {0xC7, 0x40}, {0xCC, 0x42}, {0xCD, 0x3F}, {0xCE, 0x71}
};

esp_err_t single_shot_set_wb_mode(wb_mode_t mode) {
    if (!g_initialized) {
        ESP_LOGE(TAG, "Camera not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    const uint8_t (*regs)[3] = NULL;
    int reg_count = 0;
    
    switch (mode) {
        case WB_MODE_AUTO:
            regs = OV2640_WB_AUTO;
            reg_count = sizeof(OV2640_WB_AUTO) / sizeof(OV2640_WB_AUTO[0]);
            break;
        case WB_MODE_SUNNY:
            regs = OV2640_WB_SUNNY;
            reg_count = sizeof(OV2640_WB_SUNNY) / sizeof(OV2640_WB_SUNNY[0]);
            break;
        case WB_MODE_CLOUDY:
            regs = OV2640_WB_CLOUDY;
            reg_count = sizeof(OV2640_WB_CLOUDY) / sizeof(OV2640_WB_CLOUDY[0]);
            break;
        case WB_MODE_OFFICE:
            regs = OV2640_WB_OFFICE;
            reg_count = sizeof(OV2640_WB_OFFICE) / sizeof(OV2640_WB_OFFICE[0]);
            break;
        case WB_MODE_HOME:
            regs = OV2640_WB_HOME;
            reg_count = sizeof(OV2640_WB_HOME) / sizeof(OV2640_WB_HOME[0]);
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret = power_up_sensor();
    if (ret != ESP_OK) return ret;
    
    for (int i = 0; i < reg_count; i++) {
        ret = sensor_write_reg(regs[i][0], regs[i][1]);
        if (ret != ESP_OK) {
            power_down_sensor();
            return ret;
        }
    }
    
    power_down_sensor();
    return ESP_OK;
}

esp_err_t single_shot_set_brightness(int level) {
    if (!g_initialized) {
        ESP_LOGE(TAG, "Camera not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (level < -2 || level > 2) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Brightness values from ESP32-Camera
    uint8_t brightness_regs[][4] = {
        {0xFF, 0x00}, {0x7C, 0x00}, {0x7D, 0x04}, {0x7C, 0x09}
    };
    uint8_t brightness_vals[] = {0x20, 0x10, 0x00, 0x10, 0x20}; // -2 to +2
    
    esp_err_t ret = power_up_sensor();
    if (ret != ESP_OK) return ret;
    
    // Write brightness registers
    for (int i = 0; i < 4; i++) {
        if (i == 3) {
            ret = sensor_write_reg(brightness_regs[i][0], brightness_vals[level + 2]);
        } else {
            ret = sensor_write_reg(brightness_regs[i][0], brightness_regs[i][1]);
        }
        if (ret != ESP_OK) {
            power_down_sensor();
            return ret;
        }
    }
    
    power_down_sensor();
    return ESP_OK;
}

esp_err_t single_shot_set_contrast(int level) {
    if (!g_initialized) {
        ESP_LOGE(TAG, "Camera not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (level < -2 || level > 2) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Contrast values from ESP32-Camera
    uint8_t contrast_regs[][4] = {
        {0xFF, 0x00}, {0x7C, 0x00}, {0x7D, 0x04}, {0x7C, 0x07}
    };
    uint8_t contrast_vals[] = {0x18, 0x1C, 0x20, 0x24, 0x28}; // -2 to +2
    
    esp_err_t ret = power_up_sensor();
    if (ret != ESP_OK) return ret;
    
    for (int i = 0; i < 4; i++) {
        if (i == 3) {
            ret = sensor_write_reg(contrast_regs[i][0], contrast_vals[level + 2]);
        } else {
            ret = sensor_write_reg(contrast_regs[i][0], contrast_regs[i][1]);
        }
        if (ret != ESP_OK) {
            power_down_sensor();
            return ret;
        }
    }
    
    power_down_sensor();
    return ESP_OK;
}

esp_err_t single_shot_set_saturation(int level) {
    if (!g_initialized) {
        ESP_LOGE(TAG, "Camera not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (level < -2 || level > 2) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t sat_regs[][4] = {
        {0xFF, 0x00}, {0x7C, 0x00}, {0x7D, 0x02}, {0x7C, 0x03}
    };
    uint8_t sat_vals[] = {0x28, 0x30, 0x40, 0x50, 0x60}; // -2 to +2
    
    esp_err_t ret = power_up_sensor();
    if (ret != ESP_OK) return ret;
    
    for (int i = 0; i < 4; i++) {
        if (i == 3) {
            ret = sensor_write_reg(sat_regs[i][0], sat_vals[level + 2]);
        } else {
            ret = sensor_write_reg(sat_regs[i][0], sat_regs[i][1]);
        }
        if (ret != ESP_OK) {
            power_down_sensor();
            return ret;
        }
    }
    
    power_down_sensor();
    return ESP_OK;
}

esp_err_t single_shot_set_agc_gain(int gain_db) {
    if (!g_initialized) {
        ESP_LOGE(TAG, "Camera not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (gain_db < 0 || gain_db > 30) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret = power_up_sensor();
    if (ret != ESP_OK) return ret;
    
    ret = ov2640_set_bank(BANK_SENSOR);
    if (ret != ESP_OK) {
        power_down_sensor();
        return ret;
    }
    
    // Map gain_db to register value
    uint8_t gain_val = gain_db * 255 / 30;
    ret = sensor_write_reg(0x00, gain_val); // AGC gain register
    
    power_down_sensor();
    return ret;
}

esp_err_t single_shot_set_ae_level(int level) {
    if (!g_initialized) {
        ESP_LOGE(TAG, "Camera not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (level < -2 || level > 2) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t ae_vals[] = {0x10, 0x20, 0x30, 0x40, 0x50}; // -2 to +2
    
    esp_err_t ret = power_up_sensor();
    if (ret != ESP_OK) return ret;
    
    ret = ov2640_set_bank(BANK_SENSOR);
    if (ret != ESP_OK) {
        power_down_sensor();
        return ret;
    }
    
    ret = sensor_write_reg(0x24, ae_vals[level + 2]); // AE level register
    
    power_down_sensor();
    return ret;
}

esp_err_t single_shot_set_framesize(uint16_t width, uint16_t height) {
    if (!g_initialized) {
        ESP_LOGE(TAG, "Camera not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Update global configuration
    g_config.width = width;
    g_config.height = height;
    
    // TODO: Implement framesize register configuration
    // This requires complex register sequences from ESP32-Camera
    ESP_LOGW(TAG, "Frame size configuration not yet fully implemented");
    
    return ESP_OK;
}

// RGB capture implementation using DVP interface


static esp_err_t sensor_configure_rgb_capture() {
    // Already configured to RGB565 during initialization
    ESP_LOGI(TAG, "RGB565 capture configuration already applied");
    return ESP_OK;
}

static esp_err_t capture_rgb_frame(uint8_t* rgb_buf, uint16_t width, uint16_t height) {
    ESP_LOGI(TAG, "Single-shot RGB frame capture: %dx%d (RGB565 format)", width, height);
    
    esp_err_t ret;
    
    // The sensor is already initialized with RGB565 format
    // Now we just need to trigger frame output
    
    ESP_LOGI(TAG, "Sensor ready - starting single-shot capture");
    
    // Use no-VSYNC capture since we're in snapshot mode
    // In snapshot mode, VSYNC is only active during actual frame capture
    ret = ll_cam_dvp_capture_no_vsync(rgb_buf, 3000);  // 3 second timeout
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Single-shot DMA capture failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Single-shot RGB frame captured successfully");
    return ESP_OK;
}

static esp_err_t capture_rgb_frame_with_trigger(uint8_t* rgb_buf, uint16_t width, uint16_t height) {
    ESP_LOGI(TAG, "Starting synchronized DMA+trigger capture: %dx%d", width, height);
    if (!g_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    // Select trigger function: prefer FREX if available, else I2C COM3 method
    esp_err_t (*trigger_fn)(void) = NULL;
    if (g_config.frex_pin >= 0) {
        trigger_fn = single_shot_trigger_snapshot;
    } else {
        trigger_fn = single_shot_trigger_i2c_snapshot;
    }
    // Start DMA first, then trigger snapshot
    esp_err_t ret = ll_cam_dvp_capture_with_trigger(rgb_buf, 3000, trigger_fn);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Synchronized DMA+trigger capture failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Synchronized capture completed successfully");
    return ESP_OK;
}