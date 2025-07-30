cmake_minimum_required(VERSION 3.16.0)

# Check IDF version
if(IDF_VERSION_MAJOR LESS 5)
    message(FATAL_ERROR "This project requires ESP-IDF v5.0 or newer")
endif()

set(CMAKE_CXX_STANDARD 17)

# Suppress warnings about format specifiers
add_compile_options(-Wno-format)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# "Trim" the build. Include the minimal set of components, main, and anything it depends on.
idf_build_set_property(MINIMAL_BUILD ON)
project(esp32cam_snap)
