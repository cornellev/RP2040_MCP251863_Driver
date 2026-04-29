add_library(
    rp2040_mcp251863 STATIC
    src/mcp251863.cpp
)

target_include_directories(rp2040_mcp251863 PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(
        rp2040_mcp251863 PUBLIC
        hardware_spi
        pico_stdlib
)
