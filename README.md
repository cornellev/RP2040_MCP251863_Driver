# RP2040_MCP251863_Driver
A driver for the RP2040 to interface with the MCP251863 CANFD Transceiver + Controller

## CAN FD send/receive API

```cpp
uint8_t payload[8] = {0x01, 0x02, 0x03, 0x04};

can.send_canfd(0x123, payload, 4, true);
can.send_canfd(0x18DAF110, payload, 4, true, true);

canfd_frame_MCP251863_t frame = can.read_canfd();
if (frame.valid) {
    // frame.id, frame.data, frame.len, frame.brs, frame.esi, frame.ide, frame.fdf
}
```

`send_canfd()` and `read_canfd()` use `txFifoNum` and `rxFifoNum`, which default to 1 and 2. Use the FIFO-number overloads when a call should target a specific FIFO.

For full control over the transmit object, use `send_frame()` with `canfd_frame_MCP251863_t`. It supports standard or extended IDs, classical CAN or CAN FD, DLC/length mapping, BRS, ESI, RTR, SID11, and TEF sequence numbers. `read_frame()` decodes the receive object into the same struct, including filter hit and optional RX timestamp fields.

## Default initialization

`init()` now performs the chip bring-up sequence: reset, oscillator readiness wait, configuration mode, nominal/data bit timing, TDC, FIFO setup, accept-all RX filter, interrupt enables, transceiver normal mode, and CAN FD normal mode.

The built-in defaults assume a 40 MHz CAN clock, 500 kbit/s nominal arbitration rate, 2 Mbit/s CAN FD data rate, TX FIFO 1, RX FIFO 2, and 64-byte payload FIFOs. Use `init(config)` with `init_config_MCP251863_t` to override timing, FIFO numbers, FIFO depths, payload sizes, PLL, and TDC settings.

Named timing presets are available for common 40 MHz-clock cases:

```cpp
config.nominalBitTiming = MCP251863_BITTIMING_500K_40MHZ;
config.dataBitTiming = MCP251863_BITTIMING_2M_40MHZ;
```

Use `getStatus()` for controller-wide interrupt, bus error, error counter, bus-off, and SPI CRC status. Use `getFIFOStatus(fifo)` for FIFO full/empty, RX overflow, TX attempts exhausted, arbitration lost, aborted, and TX error flags.

Minimal validation examples live in `examples/canfd_tx_example.cpp` and `examples/canfd_rx_example.cpp`.
