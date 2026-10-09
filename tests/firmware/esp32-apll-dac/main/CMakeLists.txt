idf_component_register(SRCS "main.c" INCLUDE_DIRS "." REQUIRES esp_driver_i2c esp_driver_i2s esp_timer)

if(DEFINED APLL_DAC_RATE)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE APLL_DAC_RATE=${APLL_DAC_RATE})
endif()
if(DEFINED APLL_DAC_I2S_PORT)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE APLL_DAC_I2S_PORT=${APLL_DAC_I2S_PORT})
endif()
