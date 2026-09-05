# Edit following two lines to set component requirements (see docs)
set(COMPONENT_REQUIRES )
set(COMPONENT_PRIV_REQUIRES arduino-audio-tools 
                            ESP32-A2DP 
                            driver 
                            nvs_flash)

set(COMPONENT_SRCS "main.cpp")
set(COMPONENT_ADD_INCLUDEDIRS "")

register_component()

