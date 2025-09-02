#pragma once

#include <functional>

#include "esp_err.h"
// Required for build-time feature flags
#include "sdkconfig.h"

// Forward declare esp_camera types when using legacy driver path
#ifdef __cplusplus
extern "C" {
#endif
#if !CONFIG_SNAP_SINGLE_SHOT_LL_DVP
#include "esp_camera.h"
#endif
#ifdef __cplusplus
}
#endif

/**
 * @brief Camera control using single-shot power-efficient capture
 *
 */
class CameraCtl
{
public:
    /**
     * @brief Representation of a picture issued by CameraCtl.
     *
     */
    struct Picture
    {
        friend CameraCtl;

        /**
         * @brief: Returns a non-modifiable pointer to the RGB888 image buffer (96x96x3 bytes).
         *
         */
        const uint8_t* image() const;
        
        /**
         * @brief: return the size of the RGB888 buffer (always 96*96*3 = 27,648 bytes).
         *
         */
        size_t size() const;

        /**
         * @brief: return the width of the image (always 96).
         */
        uint16_t width() const;
        
        /**
         * @brief: return the height of the image (always 96).
         */
        uint16_t height() const;

    private:
        Picture();
        ~Picture();
        uint8_t* rgb_data;  // RGB888 data from single-shot camera
        bool owns_data;     // True if we need to free the data ourselves

        // When using legacy esp32-camera path, we keep the framebuffer to return later
#if !CONFIG_SNAP_SINGLE_SHOT_LL_DVP
        camera_fb_t* fb;
#endif

        // Helper to convert RGB565 buffer to RGB888
        static void convert_rgb565_to_rgb888(const uint8_t* rgb565, uint8_t* rgb888, int width, int height);
    };

    /**
     * @brief Tag descriptor of the class, useful for logging.
     *
     */
    static constexpr const char* TAG = "camera_ctl";

    /**
     * @brief Build a CameraCtl object
     *
     * @note It cannot be instantiated before "app_main" is called.
     *
     */
    CameraCtl();

    /**
     * @brief Check if camera was successfully initialized
     *
     * @return true if camera is ready, false otherwise
     */
    bool is_initialized() const;

    /**
     * @brief Capture an image and may "do" something with it
     *
     * @param f is a "FunctionObject" that takes an image as an argument and returns "void".
     *
     */
    void capture_do(std::function<void(const Picture &)>);
private:
    bool initialized;
};
