idf_component_register(SRCS "main.c"
                            "bt_app_av.c"
                            "bt_app_core.c"
                            "audio_output_i2s.cpp"
                            "codec_config/codec_config.c"
                    INCLUDE_DIRS "." "codec_config"
                    PRIV_REQUIRES bt driver esp_ringbuf nvs_flash
                    )

# cfint514 sources kept byte-identical; their %u/uint32_t logs
# trip v5.1.4 -Werror=format - relax only for this component.
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error=format)
