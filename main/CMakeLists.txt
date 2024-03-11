idf_component_register(SRCS "save_data.cpp" "servo.cpp" "web.cpp" "WiFi.cpp" "oled_display.cpp" "sd_card.cpp" "main.cpp" "main_config.cpp"
                    INCLUDE_DIRS ".")
target_compile_options(${COMPONENT_LIB} PRIVATE -std=gnu++23)