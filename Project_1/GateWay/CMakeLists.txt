# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

set(EXTRA_COMPONENT_DIRS ${CMAKE_CURRENT_LIST_DIR}/Libs ${CMAKE_CURRENT_LIST_DIR}/components) 
List(APPEND EXTRA_COMPONENT_DIRS components/lvgl_esp32_driver components/lvgl_esp32_driver/lvgl_touch components/lvgl_esp32_driver/lvgl_tft)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(GateWay)