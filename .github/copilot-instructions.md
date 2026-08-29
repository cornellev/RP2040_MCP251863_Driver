This repository is a minimal C++ driver for the Microchip MCP251863 CAN FD controller/transceiver targeting the RP2040 (Pico SDK).

Key facts for an AI coding agent working in this repo
- Big picture: The repo exposes a single class, `MCP251863` (header: `include/mcp251863.h`, implementation: `src/mcp251863.cpp`).
  - It implements a blocking SPI-based driver: low-level register read/write, FIFO management, CAN FD frame packing/unpacking, and basic status helpers.
  - Examples in `examples/` show how the library is intended to be used from a Pico program (use `spi_init`, set GPIO SPI functions, call `can.init()` then `send_canfd()` / `read_frame()`).

- Build & integration: `CMakeLists.txt` defines a library target `rp2040_mcp251863` that expects to be linked into a Pico project.
  - This repo does NOT contain a full Pico project CMake wrapper for the examples. To compile the examples, add this repo as a subdirectory or link `rp2040_mcp251863` from your Pico project and link `hardware_spi` + `pico_stdlib` (examples show `spi_init(spi0, 10*1000*1000)` usage).
  - Minimal CMake usage (paste into your Pico project's CMake):
    - add_subdirectory(path/to/RP2040_MCP251863_Driver)
    - target_link_libraries(<your-target> PRIVATE rp2040_mcp251863 hardware_spi pico_stdlib)

- Important API & conventions (concrete examples)
  - Construction: `MCP251863 can(spi0, PIN_CS, PIN_STBY);` (driver stores CS and STBY GPIOs and toggles them directly).
  - Init: `can.init()` uses a built-in default config (see `default_init_config()` in `src/mcp251863.cpp`) tuned for a 40 MHz CAN clock and 500k/2M bit timings.
  - Defaults: `txFifoNum`=1, `rxFifoNum`=2, payload sizes default to 64 bytes. Use `init(config)` to override.
  - Sending: `can.send_canfd(id, data, len, brs, extended)` (convenience) or `can.send_frame(fifo, frame)` for full control. Examples: `examples/canfd_tx_example.cpp`.
  - Receiving: `auto frame = can.read_frame(); if (frame.valid) { ... }` (see `examples/canfd_rx_example.cpp`).
  - Status: `getStatus()` and `getFIFOStatus(fifo)` return controller and FIFO state bitfields.

- Coding patterns and gotchas for patches
  - SPI and GPIO operations are synchronous and blocking (pico SDK blocking SPI calls). Avoid introducing async/ISR assumptions unless you also add concurrency protections.
  - The driver manipulates CS with raw `gpio_put` and small inline `nop` delays — keep that sequence intact when editing read/write paths.
  - Many methods return `int` as boolean success (1) / failure (0). Preserve this error-signalling when changing public methods.
  - Address packing: read/write commands build a 12-bit address/command word (see `writeAddr` / `readAddr`). Be careful when changing bit layout.
  - Frame packing/unpacking is done with fixed endianness helpers (`store_word`, `load_word`) and fixed header layout — tests/examples depend on that exact layout.

- Where to look when implementing features or debugging
  - Public API & constants: `include/mcp251863.h` (types, enums, public methods)
  - Core logic: `src/mcp251863.cpp` (init sequence, FIFO logic, create_message_obj, encode/decode functions)
  - Usage examples: `examples/canfd_tx_example.cpp`, `examples/canfd_rx_example.cpp` (show SPI init, pin wiring, main loop behaviour)
  - Defaults and timing presets: `default_init_config()` in `src/mcp251863.cpp` — change here to alter defaults.

- Tests & CI
  - There are no unit tests or CI. Keep changes small and validate on hardware or by compiling into a Pico project.

- Good first edits the agent can do safely
  - Add a small example CMakeLists.txt that builds the examples when the repo is used as the top-level project (non-invasive and optional).
  - Add lightweight defensive checks around `readAddr`/`writeAddr` buffer sizes and return values.
  - Add a README section that documents how to link the library into a Pico CMake project (copy the suggested CMake snippet above).

If anything in this file is unclear or you want the agent to expand any section (for example, generate a top-level example CMake to build the examples), tell me which section to expand or any preferred style and I will update the file.
