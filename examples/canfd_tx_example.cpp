#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "mcp251863.h"

// adjust these pins to match your board
static constexpr uint SPI_SCK  = 2;
static constexpr uint SPI_TX   = 3;
static constexpr uint SPI_RX   = 4;
static constexpr uint CS_PIN   = 5;
static constexpr uint STBY_PIN = 6;

int main() {
    stdio_init_all();
    sleep_ms(2000);  // wait for USB serial to connect
    printf("TX example starting\n");

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
    printf("init ok\n");

    // known payload: 8 bytes counting up from 0
    uint8_t data[8] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
    uint32_t id = 0x123;
    int count = 0;

    for (;;) {
        // put the frame count in the first byte so you can verify sequence on the receiver
        data[0] = (uint8_t)(count & 0xFF);

        int ok = mcp.send_canfd(id, data, sizeof(data), true);
        if (ok) {
            printf("sent frame %d  id=0x%03X  data[0]=%02X\n", count, id, data[0]);
        } else {
            Status s = mcp.getStatus();
            printf("send failed  bus_off=%d  tx_err=%d  tx_count=%d\n",
                s.bus_off, s.tx_error_passive, s.tx_error_count);
        }

        count++;
        sleep_ms(500);
    }

    return 0;
}