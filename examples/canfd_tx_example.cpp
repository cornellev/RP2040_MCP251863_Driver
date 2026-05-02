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

    uint8_t payload[8] = {0xCA, 0xFE, 0x25, 0x18, 0x63, 0x00, 0x00, 0x01};
    while (true) {
        if (can.send_canfd(0x123, payload, sizeof(payload), true)) {
            printf("sent CAN FD frame id=0x123 len=8 brs=1\n");
        }
        else {
            status_MCP251863_t status = can.getStatus();
            printf("send failed: bus_off=%d tx_warn=%d rx_warn=%d tec=%u rec=%u\n",
                status.bus_off,
                status.tx_error_warning,
                status.rx_error_warning,
                status.tx_error_count,
                status.rx_error_count);
        }
        sleep_ms(1000);
    }
}
