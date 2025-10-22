# MAX31856 library sources
add_library(max31856_lib OBJECT)

target_sources(max31856_lib PRIVATE
    ${CMAKE_SOURCE_DIR}/MAX31856/max31856/max31856.c
)

target_include_directories(max31856_lib PUBLIC
    ${CMAKE_SOURCE_DIR}/MAX31856/max31856
    ${CMAKE_SOURCE_DIR}/Core/Inc
    ${CMAKE_SOURCE_DIR}/Drivers/STM32H5xx_HAL_Driver/Inc
    ${CMAKE_SOURCE_DIR}/Drivers/STM32H5xx_HAL_Driver/Inc/Legacy
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32H5xx/Include
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include
)

target_compile_definitions(max31856_lib PUBLIC
    STM32H563xx
    USE_HAL_DRIVER
)
