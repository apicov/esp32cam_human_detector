idf_component_register(SRCS "main.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES camera_ctl wifi_station mqtt_client nvs_flash fatfs)
