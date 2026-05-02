#ifndef MCP251863_H
#define MCP251863_H

#include <stdlib.h>
#include <string.h>

#include <cstdio>

#include "hardware/spi.h"
#include "pico/stdlib.h"

// Constants and magic numbers
const uint MCP251863_BAUD_RATE = 160000000;

// Enums for various device modes/configs

// write mode
enum wm_MCP251863_t { WM_MCP_NORM = 0, WM_MCP_CRC = 1, WM_MCP_SAFE = 2 };

// read mode
enum rm_MCP251863_t { RM_MCP_NORM = 0, RM_MCP_CRC = 1 };

// command type
enum cmd_MCP251863_t : uint8_t {
    CMD_MCP_RESET = 0b0000,  // reset
    CMD_MCP_READA = 0b0011,  // read address
    CMD_MCP_WRITA = 0b0010,  // write address
    CMD_MCP_RDACR = 0b1011,  // read address crc
    CMD_MCP_WRACR = 0b1010,  // write adress crc
    CMD_MCP_WRASF = 0b1100   // write address safe
};

// controller mode
enum cmode_MCP251863_t : uint8_t {
    CMODE_MCP_CONF      = 0b100,  // config
    CMODE_MCP_CFD_NORM  = 0b000,  // canfd normal
    CMODE_MCP_C2_NORM   = 0b110,  // can2 normal
    CMODE_MCP_SLP       = 0b001,  // sleep
    CMODE_MCP_LIST_ONLY = 0b011,  // listen only
    CMODE_MCP_RESTR_OP  = 0b111,  // restricted operation
    CMODE_MCP_INT_LOOP  = 0b010,  // internal loopback
    CMODE_MCP_EXT_LOOP  = 0b101   // external loopback
};

// tranceiver mode
enum tmode_MCP251863_t { TMODE_MCP_STBY = 1, TMODE_MCP_NORM = 0 };

// tx fifo retransmit mode
enum tx_retran_mode_MCP251863_t : uint8_t {
    TXRET_MCP_NONE  = 0b00,
    TXRET_MCP_THREE = 0b01,
    TXRET_MCP_UNLIM = 0b10
};

enum fifo_int_mode_MCP251863_t : uint8_t {
    FIFO_INT_MCP_NFNE  = 0b00000001,  // fifo not full (TX), fifo not empty (RX)
    FIFO_INT_MCP_HFHE_ = 0b00000010,  // fifo half full(TX), fifo half empty (RX)
    FIFO_INT_MCP_FFEE  = 0b00000100,  // fifo full (TX), fifo empty (RX),
    FIFO_INT_MCP_OVFL  = 0b00001000,  // fifo overflow (RX),
    FIFO_INT_MCO_TXAT  = 0b00010000,  // transmits exhausted,
};

enum reg_addr_MCP251863_t : uint16_t {
    REG_MCP_OSC        = 0xE00,
    REG_MCP_IOCON      = 0xE04,
    REG_MCP_CRC        = 0xE08,
    REG_MCP_ECCCON     = 0xE0C,
    REG_MCP_ECCSTAT    = 0xE10,
    REG_MCP_DEVID      = 0xE14,
    REG_MCP_C1CON      = 0x0,
    REG_MCP_C1NBTCFG   = 0x4,
    REG_MCP_C1DBTCFG   = 0x8,
    REG_MCP_C1TDC      = 0xC,
    REG_MCP_C1TBC      = 0x10,
    REG_MCP_C1TSCON    = 0x14,
    REG_MCP_C1VEC      = 0x18,
    REG_MCP_C1INT      = 0x1C,
    REG_MCP_C1RXIF     = 0x20,
    REG_MCP_C1TXIF     = 0x24,
    REG_MCP_C1RXOVIF   = 0x28,
    REG_MCP_C1TXATIF   = 0x2C,
    REG_MCP_C1TXREQ    = 0x30,
    REG_MCP_C1TREC     = 0x34,
    REG_MCP_C1BDIAGx   = 0x38,  // 4 addresses between each
    REG_MCP_C1TEFCON   = 0x40,
    REG_MCP_C1TEFSTA   = 0x44,
    REG_MCP_C1TEFUA    = 0x48,
    REG_MCP_C1TXQCON   = 0x50,
    REG_MCP_C1TXQSTA   = 0x54,
    REG_MCP_C1TXQUA    = 0x58,
    REG_MCP_C1FIFOCONx = 0x5C,   // 12 addresses between each
    REG_MCP_C1FIFOSTAx = 0x60,   // 12 addresses between each
    REG_MCP_C1FIFOUAx  = 0x64,   // 12 addresses between each
    REG_MCP_C1FLTCONx  = 0x1D0,  // 4 addresses between each
    REG_MCP_C1FLTOBJx  = 0x1F8,  // 8 addresses between each
    REG_MCP_C1MASKx    = 0x1F4   // 8 addresses between each
};

enum fifo_mode_MCP251863_t : uint8_t { FIFO_MODE_MCP_TX = 1, FIFO_MODE_MCP_RX = 0 };

// payload size for fifo
enum pl_size_MCP251863_t : uint8_t {
    PL_SIZE_MCP_0  = 0b0000,
    PL_SIZE_MCP_1  = 0b0001,
    PL_SIZE_MCP_2  = 0b0010,
    PL_SIZE_MCP_3  = 0b0011,
    PL_SIZE_MCP_4  = 0b0100,
    PL_SIZE_MCP_5  = 0b0101,
    PL_SIZE_MCP_6  = 0b0110,
    PL_SIZE_MCP_7  = 0b0111,
    PL_SIZE_MCP_8  = 0b1000,
    PL_SIZE_MCP_12 = 0b1001,
    PL_SIZE_MCP_16 = 0b1010,
    PL_SIZE_MCP_20 = 0b1011,
    PL_SIZE_MCP_24 = 0b1100,
    PL_SIZE_MCP_32 = 0b1101,
    PL_SIZE_MCP_48 = 0b1110,
    PL_SIZE_MCP_64 = 0b1111
};

enum err_MCP251863_t : uint8_t { ERR_MCP_FIFO_FULL = 0 };

enum int_en_MCP251863_t : uint32_t {
    INT_EN_MCP_TXIF     = 0b00000000000000000000000000000001,
    INT_EN_MCP_RXIF     = 0b00000000000000000000000000000010,
    INT_EN_MCP_TBCIF    = 0b00000000000000000000000000000100,
    INT_EN_MCP_MODIF    = 0b00000000000000000000000000001000,
    INT_EN_MCP_TEFIF    = 0b00000000000000000000000000010000,
    INT_EN_MCP_ECCIF    = 0b00000000000000000000000100000000,
    INT_EN_MCP_SPICRCIF = 0b00000000000000000000001000000000,
    INT_EN_MCP_TXATIF   = 0b00000000000000000000010000000000,
    INT_EN_MCP_RXOVIF   = 0b00000000000000000000100000000000,
    INT_EN_MCP_SERRIF   = 0b00000000000000000001000000000000,
    INT_EN_MCP_CERRIF   = 0b00000000000000000010000000000000,
    INT_EN_MCP_WAKIF    = 0b00000000000000000100000000000000,
    INT_EN_MCP_IVMIF    = 0b00000000000000001000000000000000,
    INT_EN_MCP_TXIE     = 0b00000000000000010000000000000000,
    INT_EN_MCP_RXIE     = 0b00000000000000100000000000000000,
    INT_EN_MCP_TBCIE    = 0b00000000000001000000000000000000,
    INT_EN_MCP_MODIE    = 0b00000000000010000000000000000000,
    INT_EN_MCP_TEFIE    = 0b00000000000100000000000000000000,
    INT_EN_MCP_ECCIE    = 0b00000001000000000000000000000000,
    INT_EN_MCP_SPICRCIE = 0b00000010000000000000000000000000,
    INT_EN_MCP_TXATIE   = 0b00000100000000000000000000000000,
    INT_EN_MCP_RXOVIE   = 0b00001000000000000000000000000000,
    INT_EN_MCP_SERRIE   = 0b00010000000000000000000000000000,
    INT_EN_MCP_CERRIE   = 0b00100000000000000000000000000000,
    INT_EN_MCP_WAKIE    = 0b01000000000000000000000000000000,
    INT_EN_MCP_IVMIE    = 0b10000000000000000000000000000000
};

enum io_num_MCP251863_t { IO_MCP_INT0 = 0, IO_MCP_INT1 = 1 };

enum iomode_MCP251863_t { IOMODE_MCP_INT = 0, IOMODE_MCP_GPIO_OUT = 1, IOMODE_MCP_GPIO_IN = 2 };

enum msgtype_MCP251863_t { CAN_BASE_MCP = 0, CAN_FD_BASE_MCP = 1, CAN_EXT = 2, CAN_FD_EXT = 3 };

// Structs

struct bit_timing_MCP251863_t {
    uint8_t brp;    // Baud rate prescaler (0 = divide by 1)
    uint8_t tseg1;  // Time segment 1 (propagation + phase1)
    uint8_t tseg2;  // Time segment 2 (phase2)
    uint8_t sjw;    // Synchronization jump width
};

struct canfd_frame_MCP251863_t {
    uint32_t id;           // CAN ID (11-bit for std, 29-bit for ext)
    uint8_t dlc;           // Data length code (register value)
    uint8_t len;           // Actual payload length in bytes
    bool ide;              // Extended ID flag
    bool fdf;              // FD frame flag
    bool brs;              // Bit rate switch flag
    bool rtr;              // Remote transmission request
    bool esi;              // Error status indicator
    bool sid11;            // SID11 field for CAN-FD base frames
    bool valid;            // Frame contains valid data
    uint8_t filter_hit;    // Filter index that matched (RX only)
    uint32_t sequence;     // Sequence number (TX only)
    bool timestamp_valid;  // Timestamp field is populated
    uint32_t timestamp;    // Received timestamp (RX only, if enabled)
    uint8_t data[64];      // Payload bytes
};

struct init_config_MCP251863_t {
    int enablePll;          // Enable PLL (multiply oscillator by 10)
    int sclkDiv2;           // Divide SCLK by 2
    int enableTdc;          // Enable Transmitter Delay Compensation
    int rxTimestampEnable;  // Capture timestamp in received frames
    uint8_t tdcOffset;      // TDC offset (SSP position in TQs)
    uint8_t txFifo;         // TX FIFO number (1-31)
    uint8_t rxFifo;         // RX FIFO number (1-31, must differ from txFifo)
    uint8_t txFifoDepth;    // TX FIFO depth (1-32 messages)
    uint8_t rxFifoDepth;    // RX FIFO depth (1-32 messages)
    pl_size_MCP251863_t txPayloadSize;
    pl_size_MCP251863_t rxPayloadSize;
    bit_timing_MCP251863_t nominalBitTiming;
    bit_timing_MCP251863_t dataBitTiming;
};

struct status_MCP251863_t {
    uint32_t interrupt_flags;
    uint32_t rx_if;
    uint32_t tx_if;
    uint32_t rx_overflow_if;
    uint32_t tx_attempt_if;
    uint32_t trec;
    uint32_t bdiag0;
    uint32_t bdiag1;
    uint32_t crc;
    bool bus_off;
    bool tx_error_passive;
    bool rx_error_passive;
    bool tx_error_warning;
    bool rx_error_warning;
    bool error_warning;
    uint8_t tx_error_count;
    uint8_t rx_error_count;
    bool spi_crc_format_error;
    bool spi_crc_error;
};

struct fifo_status_MCP251863_t {
    uint8_t fifo_index;
    bool tx_aborted;
    bool tx_lost_arbitration;
    bool tx_error;
    bool tx_attempts_exhausted;
    bool rx_overflow;
    bool empty_or_full;
    bool half_empty_or_half_full;
    bool not_full_or_not_empty;
};

// Bit timing presets for a 40 MHz CAN clock.
// Bit time = (BRP+1) * (TSEG1+TSEG2+3) / Fsys
//   500 K: (0+1)*(62+15+3)/40e6 = 80/40e6  = 2 us  → 500 Kbit/s, sample point ~79%
//     2 M: (0+1)*(14+ 3+3)/40e6 = 20/40e6  = 500 ns → 2 Mbit/s,  sample point ~75%
static const bit_timing_MCP251863_t MCP251863_BITTIMING_500K_40MHZ = {0, 62, 15, 15};
static const bit_timing_MCP251863_t MCP251863_BITTIMING_2M_40MHZ   = {0, 14, 3, 3};

// Auxiliary methods
int create_message_obj(
    uint8_t* dst,
    const uint8_t* data,
    msgtype_MCP251863_t msgtype,
    pl_size_MCP251863_t plSize,
    uint32_t id,
    int brsEn);
int create_message_obj(uint8_t* dst, const canfd_frame_MCP251863_t& frame, size_t* objectSize);

// Main class
class MCP251863 {
   private:
    spi_inst_t* spi;
    uint CSPin, STBYPin;
    int setCS(uint state);

   public:
    wm_MCP251863_t writeMode;
    rm_MCP251863_t readMode;
    uint8_t txFifoNum;
    uint8_t rxFifoNum;
    int rxTimestampEnabled;

    MCP251863(spi_inst_t* ispi, uint iCSPin, uint iSTBYPin);

    int init();
    int init(const init_config_MCP251863_t& config);
    int reset();
    int setBitTiming(bit_timing_MCP251863_t nominalTiming, bit_timing_MCP251863_t dataTiming);

    int readAddr(uint16_t startAddr, uint8_t* dst, size_t len);
    int writeAddr(uint16_t startAddr, uint8_t* data, size_t len);

    int initGPFIFO(
        uint8_t fifoNum,
        fifo_mode_MCP251863_t fifoMode,
        pl_size_MCP251863_t plSize,
        uint8_t fSize,
        uint8_t prioNum,
        tx_retran_mode_MCP251863_t retranMode,
        fifo_int_mode_MCP251863_t* intFlagArray,
        size_t intFlagSize);

    int initTEFIFO(uint8_t fSize, fifo_int_mode_MCP251863_t* intFlagArray, size_t intFlagSize);

    // not important for current purposes
    // int initMask();

    int initFilter(uint8_t filNum, uint8_t fifoNum, uint16_t canSID);

    int pushTXFIFO(uint8_t fifoNum, uint8_t* data, size_t pSize);
    int reqSendTXFIFO(uint8_t fifoNum);
    int popRXFIFO(uint8_t fifoNum, uint8_t* dst, size_t pSize);

    int send_canfd(
        uint32_t id, const uint8_t* data, size_t len, bool brs, bool extended_id = false);
    int send_canfd(
        uint8_t fifoNum,
        uint32_t id,
        const uint8_t* data,
        size_t len,
        bool brs,
        bool extended_id = false);
    int send_frame(const canfd_frame_MCP251863_t& frame);
    int send_frame(uint8_t fifoNum, const canfd_frame_MCP251863_t& frame);

    canfd_frame_MCP251863_t read_canfd();
    canfd_frame_MCP251863_t read_canfd(uint8_t fifoNum);
    canfd_frame_MCP251863_t read_frame();
    canfd_frame_MCP251863_t read_frame(uint8_t fifoNum);

    int setContMode(cmode_MCP251863_t mode);
    int setTranMode(tmode_MCP251863_t mode);

    int setInterrupts(int_en_MCP251863_t* intEnArray, size_t intEnSize);
    int setPinMode(io_num_MCP251863_t pin, iomode_MCP251863_t mode);

    status_MCP251863_t getStatus();
    fifo_status_MCP251863_t getFIFOStatus(uint8_t fifoNum);

    int getTXCode();
    int getRXCode();
    int getFLTCode();
    int getICode();
};

#endif
