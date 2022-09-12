add_library(bme680
    bme680.c
)

target_include_directories(bme680 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})