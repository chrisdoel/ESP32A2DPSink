idf_component_register(SRCS "example.cpp" "A2DPcore.cpp" "A2DPav.cpp" "A2DPMain.cpp" "A2DPav.cpp"
                            "A2DPcore.cpp"
                REQUIRES bt esp_common freertos hal log nvs_flash
                INCLUDE_DIRS ".")
