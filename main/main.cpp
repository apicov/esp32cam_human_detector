#include <cstdint>
#include <cstdio>

// ESP-IDF
#include <esp_system.h>
#include <esp_log.h>
#include <esp_heap_caps.h>
#include <esp_task_wdt.h>

// FreeRTOS
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

// ESP-IDF components
#include <driver/gpio.h>
#include <mbedtls/base64.h>
#include <nvs_flash.h>

// Local components
#include <camera_ctl.hpp>
#include <MQTTClient.hpp>
#include <WiFiStation.hpp>

// Tensorflow lite micro
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/system_setup.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/core/c/common.h"
#include "tensorflow/lite/micro/micro_log.h"

// human - no human classifier (quantized version)
#include "minisqueezenet_96_quantized.h"

// utility functions for images
#include "image_utils.h"

// shorten CONFIG names
#define CONF(name) CONFIG_SNAP_ ## name

/* prototypes */
void camera_task(void *p);
void start_mqtt_client();
bool run_inference(uint8_t* rgb_image);

/* globals */
const char *TAG = "main";
MQTTClient *mqtt = nullptr;
QueueHandle_t camera_evt_queue = nullptr;  // FreeRTOS queue for camera trigger events
static bool g_sd_mounted = false;

//tf lite micro
const tflite::Model* tflu_model = nullptr;
tflite::MicroInterpreter* tflu_interpreter = nullptr;
TfLiteTensor* tflu_i_tensor = nullptr;
TfLiteTensor* tflu_o_tensor = nullptr;
constexpr int tensor_arena_size = 750000;  // 750KB - needed for the model based on AllocateTensors error
uint8_t *tensor_arena = nullptr;
float tflu_scale = 0.0f;
int32_t tflu_zeropoint = 0;

// sdcard
#include "driver/sdmmc_host.h"
#include "sdmmc_cmd.h"
#include "esp_vfs_fat.h"


esp_err_t initi_sd_card(const char *mount_point, sdmmc_card_t **card)
{  
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    // Lower SD clock to improve stability on AI-Thinker wiring
    host.max_freq_khz = 10000; // 10 MHz
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    // AI-Thinker ESP32-CAM: use 1-bit bus to avoid GPIO4 conflict (flash LED)
    slot_config.width = 1;             // use CMD=15, CLK=14, D0=2 only
    slot_config.clk = GPIO_NUM_14;
    slot_config.cmd = GPIO_NUM_15;
    slot_config.d0  = GPIO_NUM_2;
    // Enable internal pull-ups on required lines (weak but helps on AI-Thinker)
    gpio_set_pull_mode(GPIO_NUM_15, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_NUM_2, GPIO_PULLUP_ONLY);
    // Optional: GPIO4 not used in 1-bit, but keep pulled up
    gpio_set_pull_mode(GPIO_NUM_4, GPIO_PULLUP_ONLY);
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024,
        .disk_status_check_enable = false
    };
    
    return esp_vfs_fat_sdmmc_mount(
      mount_point, &host, &slot_config, &mount_config, card);
}

void init_tensorflow_model()
{
    ESP_LOGI(TAG, "=== Memory Before TensorFlow Init ===");
    ESP_LOGI(TAG, "Free PSRAM: %zu bytes", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    ESP_LOGI(TAG, "Free internal RAM: %zu bytes", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    ESP_LOGI(TAG, "Tensor arena size needed: %d bytes", tensor_arena_size);
    
    tflite::InitializeTarget();
    
    tflu_model = tflite::GetModel(minisqueezenet_96_tflite);
    if (tflu_model->version() != TFLITE_SCHEMA_VERSION) {
        ESP_LOGE(TAG, "Model version %d not equal to supported version %d.",
                 tflu_model->version(), TFLITE_SCHEMA_VERSION);
        return;
    }
    
    // Allocate tensor arena directly in PSRAM for large models
    tensor_arena = (uint8_t*) heap_caps_malloc(tensor_arena_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    ESP_LOGI(TAG, "Allocated tensor arena in PSRAM");
    if (tensor_arena == nullptr) {
        ESP_LOGE(TAG, "Failed to allocate tensor arena");
        return;
    }
    
    // Add all required operations for the quantized model
    static tflite::MicroMutableOpResolver<15> micro_op_resolver;
    micro_op_resolver.AddConv2D();
    micro_op_resolver.AddMaxPool2D();
    micro_op_resolver.AddReshape();
    micro_op_resolver.AddFullyConnected();
    micro_op_resolver.AddSoftmax();
    micro_op_resolver.AddDequantize();
    micro_op_resolver.AddQuantize();
    micro_op_resolver.AddConcatenation();
    micro_op_resolver.AddAdd();
    micro_op_resolver.AddMul();
    micro_op_resolver.AddTranspose();
    micro_op_resolver.AddMean();
    micro_op_resolver.AddLogistic();
    micro_op_resolver.AddRelu();  // May be needed for activations
    micro_op_resolver.AddPad();   // Often needed for convolutions
    
    static tflite::MicroInterpreter static_interpreter(
        tflu_model, micro_op_resolver, tensor_arena, tensor_arena_size);
    tflu_interpreter = &static_interpreter;
    
    TfLiteStatus allocate_status = tflu_interpreter->AllocateTensors();
    if (allocate_status != kTfLiteOk) {
        ESP_LOGE(TAG, "AllocateTensors() failed");
        ESP_LOGE(TAG, "Model requires too much memory for ESP32");
        ESP_LOGE(TAG, "Arena size: %d, but model needs ~700KB", tensor_arena_size);
        return;
    }
    
    tflu_i_tensor = tflu_interpreter->input(0);
    tflu_o_tensor = tflu_interpreter->output(0);
    
    tflu_scale = tflu_i_tensor->params.scale;
    tflu_zeropoint = tflu_i_tensor->params.zero_point;
    
    ESP_LOGI(TAG, "TensorFlow Lite model loaded successfully");
    ESP_LOGI(TAG, "Input shape: [%d,%d,%d,%d]", 
             tflu_i_tensor->dims->data[0], tflu_i_tensor->dims->data[1],
             tflu_i_tensor->dims->data[2], tflu_i_tensor->dims->data[3]);
    ESP_LOGI(TAG, "Output shape: [%d,%d]",
             tflu_o_tensor->dims->data[0], tflu_o_tensor->dims->data[1]);
}

extern "C" void app_main()
{
    ESP_LOGI(TAG, "Starting application...");
    
    // Log initial memory state
    ESP_LOGI(TAG, "=== Memory Diagnosis ===");
    ESP_LOGI(TAG, "Total PSRAM: %zu bytes", heap_caps_get_total_size(MALLOC_CAP_SPIRAM));
    ESP_LOGI(TAG, "Free PSRAM: %zu bytes", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    ESP_LOGI(TAG, "Free internal RAM: %zu bytes", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    
    if (heap_caps_get_total_size(MALLOC_CAP_SPIRAM) == 0) {
        ESP_LOGE(TAG, "PSRAM NOT DETECTED!");
        ESP_LOGE(TAG, "Possible causes:");
        ESP_LOGE(TAG, "1. Wrong board - not ESP32-CAM with PSRAM");
        ESP_LOGE(TAG, "2. PSRAM hardware failure");
        ESP_LOGE(TAG, "3. Wrong GPIO configuration for PSRAM");
        ESP_LOGE(TAG, "4. Bootloader built without PSRAM support");
        ESP_LOGE(TAG, "Your SqueezeNet model needs PSRAM to work!");
    } else {
        ESP_LOGI(TAG, "PSRAM detected: %zu MB", heap_caps_get_total_size(MALLOC_CAP_SPIRAM) / 1024 / 1024);
    }

    // gpio
    gpio_set_direction(GPIO_NUM_33, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_33, 0);

    // nvs
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // SD card (1-bit SDMMC mode)
    sdmmc_card_t *card;
    ret = initi_sd_card("/sdcard", &card);
    if (ret != ESP_OK) {
        ESP_LOGW("SD_CARD", "Initialization failed: %s (continuing without SD)", esp_err_to_name(ret));
        g_sd_mounted = false;
    } else {
        sdmmc_card_print_info(stdout, card);
        g_sd_mounted = true;
        // Simple health check file
        FILE *f = fopen("/sdcard/TEST.TXT", "wb");
        if (f) {
            const char *msg = "sd ok\n";
            fwrite(msg, 1, 5, f);
            fflush(f);
            fsync(fileno(f));
            fclose(f);
            ESP_LOGI("SD_CARD", "Wrote /sdcard/TEST.TXT");
        } else {
            ESP_LOGW("SD_CARD", "Failed to create /sdcard/TEST.TXT");
        }
    }

    // WiFi
    WiFiStation::start(CONF(WIFI_SSID), CONF(WIFI_PASSWORD)).on_connect(
      [](auto _){ start_mqtt_client(); }
    );

    // queues
    camera_evt_queue = xQueueCreate(10, sizeof(uint8_t));

    // Initialize TensorFlow Lite model
    init_tensorflow_model();
    
    // Increase Task WDT timeout to tolerate ML inference bursts (ESP-IDF v5 API)
    esp_task_wdt_deinit();
    esp_task_wdt_config_t wdt_cfg = {
        .timeout_ms = 15000,
        .idle_core_mask = 0, // don't monitor IDLE tasks; only registered tasks
        .trigger_panic = true
    };
    esp_task_wdt_init(&wdt_cfg);

    // tasks
    xTaskCreatePinnedToCore(camera_task, "camera", 8192, NULL, 5, NULL, 1);  // Pin to core 1; Wi-Fi uses core 0

    ESP_LOGI(TAG, "esp32cam_snap is running");
} // end of app_main

bool run_inference(uint8_t* rgb_image)
{
    if (!tflu_interpreter || !tflu_i_tensor || !tflu_o_tensor) {
        ESP_LOGE(TAG, "TensorFlow Lite not initialized");
        return false;
    }
    
    // Convert RGB888 to channels-first format (3, 96, 96) and quantize to int8
    int8_t* input_data = tflu_i_tensor->data.int8;
    
    // Rearrange from (96, 96, 3) to (3, 96, 96) format and quantize
    for (int c = 0; c < 3; c++) {  // channels
        for (int h = 0; h < 96; h++) {  // height
            for (int w = 0; w < 96; w++) {  // width
                int src_idx = h * 96 * 3 + w * 3 + c;  // channels-last index
                int dst_idx = c * 96 * 96 + h * 96 + w;  // channels-first index
                input_data[dst_idx] = (int8_t)(rgb_image[src_idx] - 128);  // Convert [0,255] to [-128,127]
            }
        }
    }
    
    // Run inference
    TfLiteStatus invoke_status = tflu_interpreter->Invoke();
    if (invoke_status != kTfLiteOk) {
        ESP_LOGE(TAG, "Inference failed");
        return false;
    }
    
    // Get quantized output - model outputs INT8 values
    int8_t* output = tflu_o_tensor->data.int8;
    
    // Dequantize output using scale and zero_point
    float scale = tflu_o_tensor->params.scale;
    int32_t zero_point = tflu_o_tensor->params.zero_point;
    float human_confidence = (output[0] - zero_point) * scale;
    
    ESP_LOGI(TAG, "Raw output: %d, Scale: %.6f, Zero point: %d", output[0], scale, zero_point);
    ESP_LOGI(TAG, "Human confidence: %.4f (>0 = human detected)", human_confidence);
    
    // Return true if human detected (threshold at 0)
    return human_confidence > 0.0;
}


void camera_task(void *p)
{
    // Register this task to the Task WDT so esp_task_wdt_reset() works
    esp_task_wdt_add(NULL);
    // Run on core 0; ensure we yield periodically during long ops
    CameraCtl cam{};
    uint8_t cmd;

    // TODO: if the image resolution can be adjusted, then it might be
    // a good idea to make the dimensions configurable
    constexpr size_t image_size{160 * 120 * 3};
    constexpr size_t b64_size{(4 * ((image_size + 2) / 3)) + 1};

    // Allocate b64_buffer in PSRAM since it's large (76KB)
    char* b64_buffer = (char*) heap_caps_malloc(b64_size, MALLOC_CAP_SPIRAM);
    if (b64_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate b64_buffer (%zu bytes) in PSRAM", b64_size);
        return;
    }
    ESP_LOGI(TAG, "Allocated b64_buffer: %zu bytes in PSRAM", b64_size);

    // Try internal RAM first, then PSRAM
    uint8_t* image_96 = (uint8_t*) heap_caps_malloc(96 * 96 * 3, MALLOC_CAP_INTERNAL);
    if (image_96 == NULL) {
        ESP_LOGW(TAG, "Failed to allocate image_96 in internal RAM, trying PSRAM");
        image_96 = (uint8_t*) heap_caps_malloc(96 * 96 * 3, MALLOC_CAP_SPIRAM);
        if (image_96 == NULL) {
            ESP_LOGE(TAG, "Failed to allocate image_96 (%d bytes) even in PSRAM", 96 * 96 * 3);
            free(b64_buffer);
            return;
        }
        ESP_LOGI(TAG, "Allocated image_96 in PSRAM");
    } else {
        ESP_LOGI(TAG, "Allocated image_96 in internal RAM");
    }

    int cont = 0;
    

    while(1)
    {
        //wait for mqtt command
        //xQueueReceive(camera_evt_queue, &cmd, portMAX_DELAY);
        
        
        //if(mqtt && mqtt->is_connected()) {
        if(1) {
            cam.capture_do([b64_buffer, image_96, cont](const auto &pic){
                auto src = pic.image();
                auto slen = pic.size();

                ESP_LOGI(TAG, "RGB data size: %zu bytes (should be %d for 96x96x3)", slen, 96*96*3);

                // Single-shot camera already provides RGB888 96x96 data - no conversion needed!
                if (src != nullptr) {
                    // Copy RGB data directly to our ML buffer
                    memcpy(image_96, src, 96 * 96 * 3);

                    // Save to SD card if mounted
                    if (g_sd_mounted) {
                        char path[64];
                        // Use 8.3 filename to work with FATFS LFN disabled
                        snprintf(path, sizeof(path), "/sdcard/SN%06d.PPM", cont);
                        // Save final: swap R/B, no vertical flip (validated by test B)
                        snprintf(path, sizeof(path), "/sdcard/S%05d.PPM", cont % 100000);
                        saveAsPPMEx(path, image_96, 96, 96, true, false);
                    } else {
                        ESP_LOGW(TAG, "SD not mounted - skipping save");
                    }
                    
                    // Run inference to detect human
                    // For long inference, temporarily unregister this task from WDT
                    esp_task_wdt_delete(NULL);
                    bool human_detected = run_inference(image_96);
                    // Re-register after inference and feed once
                    esp_task_wdt_add(NULL);
                    esp_task_wdt_reset();
                    vTaskDelay(1);
                    
                    // Control GPIO 33 based on detection
                    gpio_set_level(GPIO_NUM_33, human_detected ? 1 : 0);
                    ESP_LOGI(TAG, "Human %s - LED %s", 
                             human_detected ? "DETECTED" : "not detected",
                             human_detected ? "ON" : "OFF");
                    // Saved above
                } else {
                    ESP_LOGE(TAG, "Failed to capture image - RGB data is null");
                }

                // Optional: Base64 encode and publish via MQTT (currently disabled)
                /*
                size_t olen;
                auto ret = mbedtls_base64_encode(
                    (uint8_t*) b64_buffer, b64_size, &olen, image_96, 96 * 96 * 3);

                if (ret == 0) {
                    mqtt->publish(CONF(MQTT_IMG_TOPIC), b64_buffer, 2, 0);
                } else {
                  ESP_LOGE(TAG, "the dest buffer is too small (%zu)"
                           ", it requires a length of %zu", b64_size, olen);
                }*/
            });

            cont++;
        }
        else
            ESP_LOGE(TAG, "MQTT not connected");

        vTaskDelay(10000 / portTICK_PERIOD_MS);  // 10 second delay to allow inference to complete
    }
}

void start_mqtt_client(){
    // reconnect if the client was created already
    if (mqtt) {
        mqtt->reconnect();
        return;
    }

    mqtt = new MQTTClient{CONF(MQTT_URI)};
    mqtt->on_connect([](auto _) { mqtt->subscribe(CONF(MQTT_CMD_TOPIC)); });
    mqtt->on_data_received([](auto data) {
        uint8_t cmd = 1; //dummy data to send to the queue
        ESP_LOGI(TAG, "Received on topic: %.*s", data->topic_len, data->topic);
        ESP_LOGI(TAG, "Received command: '%.*s'", data->data_len, data->data);

        // Check if the received message is a snap command
        if (strncmp(data->topic, CONF(MQTT_CMD_TOPIC), data->topic_len) == 0) {
            if (strncmp(data->data, "snap", data->data_len) == 0)
                xQueueSend(camera_evt_queue, &cmd, 0);
        }
    });
}
