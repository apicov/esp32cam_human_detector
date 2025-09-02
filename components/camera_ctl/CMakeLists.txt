if(CONFIG_SNAP_SINGLE_SHOT_LL_DVP)
idf_component_register(SRCS "camera_ctl.cpp" "image_utils.cpp" "single_shot_camera.c" "image_converters.c" "ll_cam_dvp.c"
                       INCLUDE_DIRS "."
                       REQUIRES "driver" "esp_jpeg" "esp_timer")
else()
idf_component_register(SRCS "camera_ctl.cpp" "image_utils.cpp" "single_shot_camera.c" "image_converters.c" "ll_cam_dvp.c"
                       INCLUDE_DIRS "."
                       REQUIRES "driver" "esp_jpeg" "esp_timer" "esp32-camera")
endif()
