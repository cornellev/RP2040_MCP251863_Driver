#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "mcp251863.h"

static constexpr uint SPI_SCK  = 2;
static constexpr uint SPI_TX   = 3;
static constexpr uint SPI_RX   = 4;
static constexpr uint CS_PIN   = 5;
static constexpr uint STBY_PIN = 6;

static void print_frame(const CanFdFrame& frame) {
    printf("id=0x%03X  len=%d  fdf=%d  brs=%d  ide=%d  esi=%d  filter=%d  data=",
        frame.id, frame.len, frame.fdf, frame.brs, frame.ide, frame.esi, frame.filter_hit);
    for (int i = 0; i < frame.len; i++) {
        printf("%02X ", frame.data[i]);
    }
    printf("\n");
}

int main() {
    stdio_init_all();
    sleep_ms(2000);
    printf("RX example starting\n");

    spi_init(spi0, MCP251863_BAUD_RATE);
    spi_set_format(spi0, 8, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);
    gpio_set_function(SPI_SCK, GPIO_FUNC_SPI);
    gpio_set_function(SPI_TX,  GPIO_FUNC_SPI);
    gpio_set_function(SPI_RX,  GPIO_FUNC_SPI);

    MCP251863 mcp(spi0, CS_PIN, STBY_PIN);

    if (!mcp.init()) {
        printf("init failed\n");
        return 1;
    }
    printf("init ok — waiting for frames\n");

    int received = 0;

    for (;;) {
        CanFdFrame frame = mcp.read_frame();

        if (frame.valid) {
            received++;
            printf("[%d] ", received);
            print_frame(frame);

            // print status every 10 frames so you can spot errors accumulating
            if (received % 10 == 0) {
                Status s = mcp.getStatus();
                printf("status: bus_off=%d  rx_err=%d  rx_overflow=0x%08X\n",
                    s.bus_off, s.rx_error_count, s.rx_overflow_if);
            }
        }

        sleep_us(100);  // yield briefly, not blocking
    }

    return 0;
}