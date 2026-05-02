#include "mcp251863.h"

#include <stdio.h>

static const uint PIN_MISO = 16;
static const uint PIN_CS = 17;
static const uint PIN_SCK = 18;
static const uint PIN_MOSI = 19;
static const uint PIN_STBY = 20;

int main() {
    stdio_init_all();

    spi_init(spi0, 10 * 1000 * 1000);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    MCP251863 can(spi0, PIN_CS, PIN_STBY);
    if (!can.init()) {
        printf("MCP251863 init failed\n");
        while (true) {
            sleep_ms(1000);
        }
    }

    while (true) {
        canfd_frame_MCP251863_t frame = can.read_frame();
        if (frame.valid) {
            printf("rx id=0x%lx %s %s dlc=%u len=%u brs=%u esi=%u filter=%u",
                (unsigned long)frame.id,
                frame.ide ? "ext" : "std",
                frame.fdf ? "fd" : "classic",
                frame.dlc,
                frame.len,
                frame.brs,
                frame.esi,
                frame.filter_hit);
            if (frame.timestamp_valid) {
                printf(" ts=%lu", (unsigned long)frame.timestamp);
            }
            printf(" data=");
            for (uint8_t i=0; i<frame.len; i++) {
                printf("%02x", frame.data[i]);
            }
            printf("\n");
        }

        fifo_status_MCP251863_t rxStatus = can.getFIFOStatus(can.rxFifoNum);
        if (rxStatus.rx_overflow) {
            printf("rx overflow on fifo %u\n", can.rxFifoNum);
        }
        sleep_ms(10);
    }
}
