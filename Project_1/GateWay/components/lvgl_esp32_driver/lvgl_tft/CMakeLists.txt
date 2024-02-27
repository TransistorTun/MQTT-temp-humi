set(SOURCES "disp_driver.c")

# Include only the source file of the selected
# display controller.
if(0)
    list(APPEND SOURCES "ili9341.c")
elseif(CONFIG_LVGL_TFT_DISPLAY_CONTROLLER_SSD1963)
    list(APPEND SOURCES "ssd1963.c")
else()
    message("DISPLAY CONTROLLER NOT DEFINED")
endif()

if(CONFIG_LVGL_TFT_DISPLAY_PROTOCOL_SPI)
    list(APPEND SOURCES "disp_spi.c")
endif()

# Print the included source files
message("SOURCES contents: " "${SOURCES}")

idf_component_register(SRCS ${SOURCES}
                       INCLUDE_DIRS .
                       REQUIRES lvgl "driver")
