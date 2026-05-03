#include "mcp251863.h"

#include <optional>

// implementation of C++23 std::to_underlying
template <typename T>
constexpr auto to_underlying(T value) -> typename std::underlying_type<T>::type {
    return static_cast<typename std::underlying_type<T>::type>(value);
}

namespace {

std::optional<PayloadSize> canfd_len_to_dlc(size_t len) {
    switch (len) {
        case 0: return PayloadSize::PL_SIZE_MCP_0;
        case 1: return PayloadSize::PL_SIZE_MCP_1;
        case 2: return PayloadSize::PL_SIZE_MCP_2;
        case 3: return PayloadSize::PL_SIZE_MCP_3;
        case 4: return PayloadSize::PL_SIZE_MCP_4;
        case 5: return PayloadSize::PL_SIZE_MCP_5;
        case 6: return PayloadSize::PL_SIZE_MCP_6;
        case 7: return PayloadSize::PL_SIZE_MCP_7;
        case 8: return PayloadSize::PL_SIZE_MCP_8;
        case 12: return PayloadSize::PL_SIZE_MCP_12;
        case 16: return PayloadSize::PL_SIZE_MCP_16;
        case 20: return PayloadSize::PL_SIZE_MCP_20;
        case 24: return PayloadSize::PL_SIZE_MCP_24;
        case 32: return PayloadSize::PL_SIZE_MCP_32;
        case 48: return PayloadSize::PL_SIZE_MCP_48;
        case 64: return PayloadSize::PL_SIZE_MCP_64;
        default: return std::nullopt;
    }
}

int create_message_obj(uint8_t* dst, const CanFdFrame& frame, size_t* objectSize);

static uint8_t canfd_dlc_to_len(uint8_t dlc) {
    switch (static_cast<PayloadSize>(dlc & 0x0F)) {
        case PayloadSize::PL_SIZE_MCP_0: return 0;
        case PayloadSize::PL_SIZE_MCP_1: return 1;
        case PayloadSize::PL_SIZE_MCP_2: return 2;
        case PayloadSize::PL_SIZE_MCP_3: return 3;
        case PayloadSize::PL_SIZE_MCP_4: return 4;
        case PayloadSize::PL_SIZE_MCP_5: return 5;
        case PayloadSize::PL_SIZE_MCP_6: return 6;
        case PayloadSize::PL_SIZE_MCP_7: return 7;
        case PayloadSize::PL_SIZE_MCP_8: return 8;
        case PayloadSize::PL_SIZE_MCP_12: return 12;
        case PayloadSize::PL_SIZE_MCP_16: return 16;
        case PayloadSize::PL_SIZE_MCP_20: return 20;
        case PayloadSize::PL_SIZE_MCP_24: return 24;
        case PayloadSize::PL_SIZE_MCP_32: return 32;
        case PayloadSize::PL_SIZE_MCP_48: return 48;
        case PayloadSize::PL_SIZE_MCP_64: return 64;
        default: return 0;
    }
}

int payload_size_to_num_bytes(PayloadSize plSize) {
    return canfd_dlc_to_len(to_underlying(plSize));
}

uint8_t can_dlc_to_len(uint8_t dlc, bool fdf) {
    if (!fdf && ((dlc & 0x0F) > 8)) {
        return 8;
    }
    return canfd_dlc_to_len(dlc);
}

uint32_t encode_bit_timing(BitTiming timing) {
    return ((uint32_t)timing.brp << 24) |
        ((uint32_t)timing.tseg1 << 16) |
        ((uint32_t)timing.tseg2 << 8) |
        timing.sjw;
}

uint32_t encode_tdc(bool enable, uint8_t offset) {
    if (!enable) {
        return 0;
    }

    return (2UL << 16) | offset;
}

InitConfig default_init_config() {
    InitConfig config{};

    config.enablePll = 0;
    config.sclkDiv2 = 0;
    config.enableTdc = 1;
    config.rxTimestampEnable = 0;
    config.tdcOffset = 6;
    config.txFifo = 1;
    config.rxFifo = 2;
    config.txFifoDepth = 7;
    config.rxFifoDepth = 7;
    config.txPayloadSize     = PayloadSize::PL_SIZE_MCP_64;
    config.rxPayloadSize     = PayloadSize::PL_SIZE_MCP_64;

    // Defaults assume a 40 MHz CAN clock: nominal 500 kbit/s, data 2 Mbit/s.
    config.nominalBitTiming = kBitTiming500K40MHz;
    config.dataBitTiming    = kBitTiming2M40MHz;

    return config;
}

uint32_t pack_id_word(const CanFdFrame& frame) {
    if (frame.ide) {
        return ((frame.id >> 18) & 0x7FF) | ((frame.id & 0x3FFFF) << 11);
    }

    uint32_t sid = frame.id & 0x7FF;
    uint32_t sid11 = (frame.sid11 || (frame.id > 0x7FF)) ? ((frame.id >> 11) & 0x01) : 0;
    return sid | (sid11 << 29);
}

uint32_t pack_control_word(const CanFdFrame& frame) {
    return ((frame.sequence & 0x7FFFFF) << 9) |
        ((uint32_t)(frame.esi ? 1 : 0) << 8) |
        ((uint32_t)(frame.fdf ? 1 : 0) << 7) |
        ((uint32_t)(frame.brs ? 1 : 0) << 6) |
        ((uint32_t)(frame.rtr ? 1 : 0) << 5) |
        ((uint32_t)(frame.ide ? 1 : 0) << 4) |
        (frame.dlc & 0x0F);
}

void store_word(uint8_t* dst, uint32_t word) {
    dst[0] = (uint8_t)(word & 0xFF);
    dst[1] = (uint8_t)((word >> 8) & 0xFF);
    dst[2] = (uint8_t)((word >> 16) & 0xFF);
    dst[3] = (uint8_t)((word >> 24) & 0xFF);
}

uint32_t load_word(const uint8_t* src) {
    return ((uint32_t)src[0]) |
        ((uint32_t)src[1] << 8) |
        ((uint32_t)src[2] << 16) |
        ((uint32_t)src[3] << 24);
}

int finalize_frame_dlc(CanFdFrame* frame) {
    if (frame->fdf) {
        auto dlc_opt = canfd_len_to_dlc(frame->len);
        if (!dlc_opt) {
            return 0;
        }
        frame->dlc = to_underlying(*dlc_opt);
        return 1;
    }

    if (frame->len > 8) {
        return 0;
    }
    frame->dlc = frame->len;
    frame->brs = 0;
    return 1;
}

int validate_tx_frame(const CanFdFrame& frame) {
    if (frame.ide && (frame.id > 0x1FFFFFFF)) {
        return 0;
    }
    if (!frame.ide && (frame.id > 0xFFF)) {
        return 0;
    }
    if (!frame.fdf && frame.brs) {
        return 0;
    }
    if (frame.len > 64) {
        return 0;
    }
    return 1;
}

CanFdFrame decode_rx_header(const uint8_t* header, bool timestampEnabled) {
    CanFdFrame frame{};

    uint32_t word0 = load_word(header);
    uint32_t word1 = load_word(header + 4);
    frame.dlc = word1 & 0x0F;
    frame.ide = (word1 & (1UL << 4)) != 0;
    frame.rtr = (word1 & (1UL << 5)) != 0;
    frame.brs = (word1 & (1UL << 6)) != 0;
    frame.fdf = (word1 & (1UL << 7)) != 0;
    frame.esi = (word1 & (1UL << 8)) != 0;
    frame.filter_hit = (word1 >> 11) & 0x1F;
    frame.sid11 = (word0 & (1UL << 29)) != 0;
    frame.len = can_dlc_to_len(frame.dlc, frame.fdf);
    frame.timestamp_valid = timestampEnabled;
    if (timestampEnabled) {
        frame.timestamp = load_word(header + 8);
    }

    if (frame.ide) {
        frame.id = ((word0 & 0x7FF) << 18) | ((word0 >> 11) & 0x3FFFF);
    }
    else {
        frame.id = (word0 & 0x7FF) | ((uint32_t)(frame.sid11 ? 1 : 0) << 11);
    }

    return frame;
}

int encode_message_object(
    uint8_t* dst,
    const uint8_t* data,
    MessageType msgtype,
    PayloadSize plSize,
    uint32_t id,
    int brsEn) {
    int num_bytes = payload_size_to_num_bytes(plSize);
    if ((num_bytes == 0) && (plSize != PayloadSize::PL_SIZE_MCP_0)) {
        return 0;
    }
    if ((num_bytes > 0) && (data == NULL)) {
        return 0;
    }

    CanFdFrame frame{};
    frame.id = id;
    frame.ide   = (msgtype == MessageType::CAN_EXT) || (msgtype == MessageType::CAN_FD_EXT);
    frame.fdf   = (msgtype == MessageType::CAN_FD_BASE_MCP) || (msgtype == MessageType::CAN_FD_EXT);
    frame.brs = brsEn != 0;
    frame.len = num_bytes;
    frame.dlc   = to_underlying(plSize);
    frame.valid = 1;

    for (int i=0; i<num_bytes; i++) {
        frame.data[i] = data[i];
    }

    size_t objectSize = 0;
    return create_message_obj(dst, frame, &objectSize);
}

int create_message_obj(uint8_t* dst, const CanFdFrame& frame, size_t* objectSize) {
    CanFdFrame txFrame = frame;
    if (!validate_tx_frame(txFrame)) {
        return 0;
    }
    if (!finalize_frame_dlc(&txFrame)) {
        return 0;
    }
    memset(dst, 0, 8 + txFrame.len);
    store_word(dst, pack_id_word(txFrame));
    store_word(dst + 4, pack_control_word(txFrame));
    for (uint8_t i=0; i<txFrame.len; i++) {
        dst[8+i] = txFrame.data[i];
    }
    if (objectSize != NULL) {
        *objectSize = 8 + txFrame.len;
    }
    return 1;
}

}  // namespace

MCP251863::MCP251863(spi_inst_t *ispi, uint iCSPin, uint iSTBYPin) {
    spi_                 = ispi;
    chipSelectPin_       = iCSPin;
    standbyPin_          = iSTBYPin;
    writeMode_           = WriteMode::WM_MCP_NORM;
    readMode_            = ReadMode::RM_MCP_NORM;
    txFifoNum_           = 1;
    rxFifoNum_           = 2;
    rxTimestampsEnabled_ = false;
}

int MCP251863::writeAddr(uint16_t startAddr, const uint8_t* data, size_t len) {
    Command cmd;
    switch (writeMode_) {
        case WriteMode::WM_MCP_NORM: cmd = Command::CMD_MCP_WRITA; break;
        case WriteMode::WM_MCP_CRC: cmd = Command::CMD_MCP_WRACR; break;
        case WriteMode::WM_MCP_SAFE: cmd = Command::CMD_MCP_WRASF; break;
        default:
            return 0;
    }
    // form message CCCC-AAAAAAAAAAAA
    uint8_t message[2];

    message[0] = (to_underlying(cmd) << 4) | (startAddr >> 8);
    message[1] = (startAddr << 4) >> 4;

    // drive CS pin low
    asm volatile("nop \n nop \n nop");
    gpio_put(chipSelectPin_, 0);
    asm volatile("nop \n nop \n nop");

    // transmit message via SPI
    spi_write_blocking(spi_, message, 2);

    // write data
    spi_write_blocking(spi_, data, len);

    // drive CS pin high, ending read cycle
    asm volatile("nop \n nop \n nop");
    gpio_put(chipSelectPin_, 1);
    asm volatile("nop \n nop \n nop");

    return 1;
}

int MCP251863::readAddr(uint16_t startAddr, uint8_t* dst, size_t len) {
    // set read command based on read mode
    Command cmd;
    uint8_t message[3];
    uint16_t crc;
    switch (readMode_) {
        case ReadMode::RM_MCP_NORM: cmd = Command::CMD_MCP_READA; break;
        case ReadMode::RM_MCP_CRC: cmd = Command::CMD_MCP_RDACR; break;
        default:
            return 0;
    }
    // form message CCCC-AAAAAAAAAAAA
    message[0] = (to_underlying(cmd) << 4) | (startAddr >> 8);
    message[1] = (startAddr << 4) >> 4;

    // if (readMode == rm_MCP251863_t::READ_CRC) {
    //     message[2] = (uint8_t)len;
    // }

    // drive CS pin low
    asm volatile("nop \n nop \n nop");
    gpio_put(chipSelectPin_, 0);
    asm volatile("nop \n nop \n nop");

    // transmit message via SPI
    spi_write_blocking(spi_, message, 2);

    // write extra bits for len if CRC mode
    // if (readMode == rm_MCP251863_t::READ_CRC) {
    //    spi_write_blocking(spi, (message)+2, 1);
    //}

    // read out data
    spi_read_blocking(spi_, 0, dst, len);

    // read out CRC
    // if (readMode == rm_MCP251863_t::READ_CRC) {
    //    spi_read16_blocking(spi, 0, &crc, 1);
    //}

    // drive CS pin high, ending read cycle
    asm volatile("nop \n nop \n nop");
    gpio_put(chipSelectPin_, 1);
    asm volatile("nop \n nop \n nop");

    // check CRC
    // later

    return 1;
}

int MCP251863::init() {
    return init(default_init_config());
}

int MCP251863::init(const InitConfig& config) {
    if ((config.txFifo < 1) || (config.txFifo > 31) ||
        (config.rxFifo < 1) || (config.rxFifo > 31) ||
        (config.txFifo == config.rxFifo) ||
        (config.txFifoDepth > 31) || (config.rxFifoDepth > 31)) {
        return 0;
    }

    uint8_t one = 1;
    uint8_t zero = 0;
    uint32_t reg = 0;

    gpio_init(chipSelectPin_);
    gpio_set_dir(chipSelectPin_, GPIO_OUT);
    gpio_put(chipSelectPin_, 1);
    gpio_init(standbyPin_);
    gpio_set_dir(standbyPin_, GPIO_OUT);
    setTransceiverMode(TransceiverMode::TMODE_MCP_STBY);

    writeMode_ = WriteMode::WM_MCP_NORM;
    readMode_  = ReadMode::RM_MCP_NORM;

    if (!reset()) {
        return 0;
    }
    sleep_ms(10);

    // Wait for oscillator stability before touching CAN timing.
    for (int i=0; i<100; i++) {
        readAddr(to_underlying(RegisterAddress::REG_MCP_OSC) + 1, &one, 1);
        if ((one & (1 << 2)) != 0) {
            break;
        }
        sleep_ms(1);
        if (i == 99) {
            return 0;
        }
    }

    if (!setControllerMode(ControllerMode::CMODE_MCP_CONF)) {
        return 0;
    }

    uint8_t osc = (config.enablePll ? 0x01 : 0x00) | (config.sclkDiv2 ? 0x10 : 0x00);
    writeAddr(to_underlying(RegisterAddress::REG_MCP_OSC), &osc, 1);
    if (config.enablePll) {
        for (int i=0; i<100; i++) {
            readAddr(to_underlying(RegisterAddress::REG_MCP_OSC) + 1, &one, 1);
            if ((one & 0x01) != 0) {
                break;
            }
            sleep_ms(1);
            if (i == 99) {
                return 0;
            }
        }
    }

    if (!setBitTiming(config.nominalBitTiming, config.dataBitTiming)) {
        return 0;
    }

    reg = encode_tdc(config.enableTdc, config.tdcOffset);
    writeAddr(to_underlying(RegisterAddress::REG_MCP_C1TDC), (uint8_t*)&reg, 4);

    txFifoNum_           = config.txFifo;
    rxFifoNum_           = config.rxFifo;
    rxTimestampsEnabled_ = config.rxTimestampEnable != 0;

    FifoInterruptFlag txFlags[] = {FIFO_INT_MCP_NFNE, FIFO_INT_MCO_TXAT};
    FifoInterruptFlag rxFlags[] = {FIFO_INT_MCP_NFNE, FIFO_INT_MCP_OVFL};
    if (!initGeneralPurposeFifo(
            txFifoNum_,
            FifoMode::FIFO_MODE_MCP_TX,
            config.txPayloadSize,
            config.txFifoDepth,
            1,
            TxRetransmitMode::TXRET_MCP_UNLIM,
            txFlags,
            2)) {
        return 0;
    }
    if (!initGeneralPurposeFifo(
            rxFifoNum_,
            FifoMode::FIFO_MODE_MCP_RX,
            config.rxPayloadSize,
            config.rxFifoDepth,
            0,
            TxRetransmitMode::TXRET_MCP_NONE,
            rxFlags,
            2)) {
        return 0;
    }
    if (rxTimestampsEnabled_) {
        uint16_t rx_fifo_addr =
            to_underlying(RegisterAddress::REG_MCP_C1FIFOCONx) + 12 * (rxFifoNum_ - 1);
        readAddr(rx_fifo_addr, &one, 1);
        one |= (1 << 5);
        writeAddr(rx_fifo_addr, &one, 1);
    }

    // Disable filter 0 while programming it, then make it accept all frames into rxFifoNum.
    uint16_t flt_ctrl_addr = to_underlying(RegisterAddress::REG_MCP_C1FLTCONx);
    writeAddr(flt_ctrl_addr, &zero, 1);
    reg = 0;
    writeAddr(to_underlying(RegisterAddress::REG_MCP_C1FLTOBJx), (uint8_t*)&reg, 4);
    writeAddr(to_underlying(RegisterAddress::REG_MCP_C1MASKx), (uint8_t*)&reg, 4);
    one = 0b10000000 | (rxFifoNum_ & 0b00011111);
    writeAddr(flt_ctrl_addr, &one, 1);

    reg = 0xFFFFFFFF;
    writeAddr(to_underlying(RegisterAddress::REG_MCP_C1RXIF), (uint8_t*)&reg, 4);
    writeAddr(to_underlying(RegisterAddress::REG_MCP_C1TXIF), (uint8_t*)&reg, 4);
    writeAddr(to_underlying(RegisterAddress::REG_MCP_C1RXOVIF), (uint8_t*)&reg, 4);
    writeAddr(to_underlying(RegisterAddress::REG_MCP_C1TXATIF), (uint8_t*)&reg, 4);

    InterruptEnable interrupts[] = {
        INT_EN_MCP_RXIE,
        INT_EN_MCP_TXIE,
        INT_EN_MCP_RXOVIE,
        INT_EN_MCP_TXATIE,
        INT_EN_MCP_CERRIE,
        INT_EN_MCP_SERRIE,
    };
    if (!setInterrupts(interrupts, sizeof(interrupts) / sizeof(interrupts[0]))) {
        return 0;
    }

    setTransceiverMode(TransceiverMode::TMODE_MCP_NORM);
    if (!setControllerMode(ControllerMode::CMODE_MCP_CFD_NORM)) {
        return 0;
    }

    for (int i=0; i<100; i++) {
        readAddr(to_underlying(RegisterAddress::REG_MCP_C1CON) + 2, &one, 1);
        if ((one >> 5) == to_underlying(ControllerMode::CMODE_MCP_CFD_NORM)) {
            return 1;
        }
        sleep_ms(1);
    }

    return 0;
}

int MCP251863::setBitTiming(BitTiming nominalTiming, BitTiming dataTiming) {
    uint32_t reg = encode_bit_timing(nominalTiming);
    writeAddr(to_underlying(RegisterAddress::REG_MCP_C1NBTCFG), (uint8_t*)&reg, 4);

    reg = encode_bit_timing(dataTiming);
    writeAddr(to_underlying(RegisterAddress::REG_MCP_C1DBTCFG), (uint8_t*)&reg, 4);

    return 1;
}

int MCP251863::reset() {
    Command cmd        = Command::CMD_MCP_RESET;
    uint8_t message[2] = {0};

    message[0] = to_underlying(cmd) << 4;

    // drive CS pin low
    asm volatile("nop \n nop \n nop");
    gpio_put(chipSelectPin_, 0);
    asm volatile("nop \n nop \n nop");

    // transmit message via SPI
    spi_write_blocking(spi_, message, 2);

    // drive CS pin high, ending read cycle
    asm volatile("nop \n nop \n nop");
    gpio_put(chipSelectPin_, 1);
    asm volatile("nop \n nop \n nop");

    return 1;
}

int MCP251863::initGeneralPurposeFifo(
    uint8_t fifoNum,
    FifoMode fifoMode,
    PayloadSize plSize,
    uint8_t fSize,
    uint8_t prioNum,
    TxRetransmitMode retranMode,
    FifoInterruptFlag* intFlagArray,
    size_t intFlagSize) {
    uint8_t buff[4];
    uint16_t addr = to_underlying(RegisterAddress::REG_MCP_C1FIFOCONx) + 12 * (fifoNum - 1);

    uint8_t intFlags = 0;
    for (int i = 0; i < intFlagSize; i++) {
        intFlags |= static_cast<uint8_t>(intFlagArray[i]);
    }

    buff[0] = intFlags | (to_underlying(fifoMode) << 7);
    buff[1] = 0b00000000;
    //assumes prioNum <= 32
    buff[2] = 0b00000000 | (to_underlying(retranMode) << 5) | prioNum;
    // assumes fSize <= 32
    buff[3] = ((to_underlying(plSize) & 0b111) << 5) | fSize;

    writeAddr(addr, buff, 4);
    return 1;
}

int MCP251863::initTransmitEventFifo(
    uint8_t fSize, FifoInterruptFlag* intFlagArray, size_t intFlagSize) {
    uint8_t buff[4];
    uint16_t addr = to_underlying(RegisterAddress::REG_MCP_C1TEFCON);

    uint8_t intFlags = 0;
    for (int i = 0; i < intFlagSize; i++) {
        intFlags |= static_cast<uint8_t>(intFlagArray[i]);
    }

    // wait for reset bit to clear
    do {
        readAddr(addr + 1, buff, 1);
    } while ((buff[0] >> 2) == 1);

    // set bytes
    buff[0] = 0b00000000 | intFlags;
    buff[1] = 0b00000000;
    buff[2] = 0b00000000;
    // assumes fSize <= 32
    buff[3] = 0b00000000 | fSize;

    writeAddr(addr, buff, 4);
    return 1;
}

int MCP251863::initTransmitQueue(
    PayloadSize plSize,
    uint8_t fSize,
    uint8_t prioNum,
    TxRetransmitMode retranMode,
    FifoInterruptFlag* intFlagArray,
    size_t intFlagSize) {
    uint8_t buff[4];
    uint16_t addr = to_underlying(RegisterAddress::REG_MCP_C1TXQCON);

    uint8_t intFlags = 0;
    for (int i = 0; i < intFlagSize; i++) {
        intFlags |= static_cast<uint8_t>(intFlagArray[i]);
    }

    // wait for reset bit to clear
    do {
        readAddr(addr + 1, buff, 1);
    } while ((buff[0] >> 2) == 1);

    // set bytes
    buff[0] = 0b00000000 | intFlags;
    buff[1] = 0b00000000;
    // assumes prioNum <= 32
    buff[2] = 0b00000000 | (to_underlying(retranMode) << 5) | prioNum;
    // assumes fSize <= 32
    buff[3] = ((to_underlying(plSize) & 0b111) << 5) | fSize;

    writeAddr(addr, buff, 4);
    return 1;
}

int MCP251863::initFilter(uint8_t fltNum, uint8_t fifoNum, uint16_t canSID) {
    uint16_t flt_addr     = to_underlying(RegisterAddress::REG_MCP_C1FLTCONx) + fltNum / 4;
    uint16_t flt_obj_addr = to_underlying(RegisterAddress::REG_MCP_C1FLTOBJx) + 8 * fltNum;

    uint8_t buff[4];

    // enable filter, assumes fifoNum <= 32
    buff[0] = 0b00000000 | fifoNum | (1 << 7);
    writeAddr(flt_addr + fltNum % 4, buff, 1);

    // Pack SID[10:0] (and SID11 in bit 11) into C1FLTOBJn bits [11:0]
    buff[0] = canSID & 0xFF;
    buff[1] = (canSID >> 8) & 0x0F;
    buff[2] = 0x00;
    buff[3] = 0x00;

    writeAddr(flt_obj_addr, buff, 4);

    return 1;
}

int MCP251863::pushTXFIFO(uint8_t fifoNum, const uint8_t* data, size_t pSize) {
    uint16_t fifo_addr = to_underlying(RegisterAddress::REG_MCP_C1FIFOCONx) + 12 * (fifoNum - 1);
    uint16_t fifo_stat_addr =
        to_underlying(RegisterAddress::REG_MCP_C1FIFOSTAx) + 12 * (fifoNum - 1);
    uint16_t fifo_point_addr =
        to_underlying(RegisterAddress::REG_MCP_C1FIFOUAx) + 12 * (fifoNum - 1);

    uint8_t buff;
    uint16_t message_addr;

    // check if fifo nonfull, return if it is. We will implement real error handling later
    readAddr(fifo_stat_addr, &buff, 1);
    if ((buff & 0b00000001) == 0) {
        return 0;
    }

    // get pointer to message object from FIFO, the addresses are only 12-bits wide?
    readAddr(fifo_point_addr, (uint8_t*)&message_addr, 2);
    message_addr += 0x400;

    // write message to addr
    writeAddr(message_addr, data, pSize);

    // increment pointer
    buff = 0b00000001;
    writeAddr(fifo_addr + 1, &buff, 1);

    return 1;
}

int MCP251863::reqSendTXFIFO(uint8_t fifoNum) {
    uint16_t addr = to_underlying(RegisterAddress::REG_MCP_C1FIFOCONx) + 12 * (fifoNum - 1);
    uint8_t buff;

    // dont know if this is needed, but wait until the fifo increments
    do {
        readAddr(addr + 1, &buff, 1);
    } while ((buff & 0b00000001) == 1);

    // set bit to request txfifo send
    buff = 0b00000010;
    writeAddr(addr + 1, &buff, 1);

    return 1;
}

int MCP251863::popRXFIFO(uint8_t fifoNum, uint8_t* dst, size_t pSize) {
    uint16_t fifo_addr = to_underlying(RegisterAddress::REG_MCP_C1FIFOCONx) + 12 * (fifoNum - 1);
    uint16_t fifo_stat_addr =
        to_underlying(RegisterAddress::REG_MCP_C1FIFOSTAx) + 12 * (fifoNum - 1);
    uint16_t fifo_point_addr =
        to_underlying(RegisterAddress::REG_MCP_C1FIFOUAx) + 12 * (fifoNum - 1);

    uint8_t buff;
    uint16_t message_addr;

    // check if fifo nonempty, if it is return
    readAddr(fifo_stat_addr, &buff, 1);
    if ((buff & 0b00000001) == 0) {
        return 0;
    }

    readAddr(fifo_point_addr, (uint8_t*)&message_addr, 2);
    message_addr += 0x400;

    // read in message
    readAddr(message_addr, dst, pSize);

    // decrement fifo
    buff = 0b00000001;
    writeAddr(fifo_addr + 1, &buff, 1);

    return 1;
}

int MCP251863::send_canfd(uint32_t id, const uint8_t* data, size_t len, bool brs, bool extended_id) {
    return send_canfd(txFifoNum_, id, data, len, brs, extended_id);
}

int MCP251863::send_canfd(uint8_t fifoNum, uint32_t id, const uint8_t* data, size_t len, bool brs, bool extended_id) {
    auto dlc_opt = canfd_len_to_dlc(len);
    if (!dlc_opt) {
        return 0;
    }
    if ((len > 0) && (data == NULL)) {
        return 0;
    }

    CanFdFrame frame{};
    frame.id = id;
    frame.ide = extended_id;
    frame.fdf = 1;
    frame.brs = brs;
    frame.len = len;
    frame.dlc = to_underlying(*dlc_opt);
    for (size_t i=0; i<len; i++) {
        frame.data[i] = data[i];
    }
    return send_frame(fifoNum, frame);
}

int MCP251863::send_frame(const CanFdFrame& frame) { return send_frame(txFifoNum_, frame); }

int MCP251863::send_frame(uint8_t fifoNum, const CanFdFrame& frame) {
    uint8_t message[72];
    size_t objectSize = 0;
    if (!create_message_obj(message, frame, &objectSize)) {
        return 0;
    }
    if (!pushTXFIFO(fifoNum, message, objectSize)) {
        return 0;
    }
    return reqSendTXFIFO(fifoNum);
}

CanFdFrame MCP251863::read_canfd() { return read_frame(rxFifoNum_); }

CanFdFrame MCP251863::read_canfd(uint8_t fifoNum) { return read_frame(fifoNum); }

CanFdFrame MCP251863::read_frame() { return read_frame(rxFifoNum_); }

CanFdFrame MCP251863::read_frame(uint8_t fifoNum) {
    CanFdFrame frame{};

    uint16_t fifo_addr = to_underlying(RegisterAddress::REG_MCP_C1FIFOCONx) + 12 * (fifoNum - 1);
    uint16_t fifo_stat_addr =
        to_underlying(RegisterAddress::REG_MCP_C1FIFOSTAx) + 12 * (fifoNum - 1);
    uint16_t fifo_point_addr =
        to_underlying(RegisterAddress::REG_MCP_C1FIFOUAx) + 12 * (fifoNum - 1);

    uint8_t buff;
    uint16_t message_addr;
    uint8_t header[12] = {0};
    size_t headerSize  = rxTimestampsEnabled_ ? 12 : 8;

    readAddr(fifo_stat_addr, &buff, 1);
    if ((buff & 0b00000001) == 0) {
        return frame;
    }

    readAddr(fifo_point_addr, (uint8_t*)&message_addr, 2);
    message_addr += 0x400;
    readAddr(message_addr, header, headerSize);
    frame = decode_rx_header(header, rxTimestampsEnabled_);

    if (frame.len > 0) {
        readAddr(message_addr + headerSize, frame.data, frame.len);
    }

    buff = 0b00000001;
    writeAddr(fifo_addr+1, &buff, 1);

    frame.valid = 1;
    return frame;
}

FifoStatus MCP251863::getFIFOStatus(uint8_t fifoNum) {
    FifoStatus status{};

    uint16_t fifo_stat_addr =
        to_underlying(RegisterAddress::REG_MCP_C1FIFOSTAx) + 12 * (fifoNum - 1);
    uint32_t reg = 0;
    readAddr(fifo_stat_addr, (uint8_t*)&reg, 4);

    status.fifo_index = (reg >> 8) & 0x1F;
    status.tx_aborted = (reg & (1UL << 7)) != 0;
    status.tx_lost_arbitration = (reg & (1UL << 6)) != 0;
    status.tx_error = (reg & (1UL << 5)) != 0;
    status.tx_attempts_exhausted = (reg & (1UL << 4)) != 0;
    status.rx_overflow = (reg & (1UL << 3)) != 0;
    status.empty_or_full = (reg & (1UL << 2)) != 0;
    status.half_empty_or_half_full = (reg & (1UL << 1)) != 0;
    status.not_full_or_not_empty = (reg & 1UL) != 0;

    return status;
}

Status MCP251863::getStatus() {
    Status status{};

    readAddr(to_underlying(RegisterAddress::REG_MCP_C1INT), (uint8_t*)&status.interrupt_flags, 4);
    readAddr(to_underlying(RegisterAddress::REG_MCP_C1RXIF), (uint8_t*)&status.rx_if, 4);
    readAddr(to_underlying(RegisterAddress::REG_MCP_C1TXIF), (uint8_t*)&status.tx_if, 4);
    readAddr(to_underlying(RegisterAddress::REG_MCP_C1RXOVIF), (uint8_t*)&status.rx_overflow_if, 4);
    readAddr(to_underlying(RegisterAddress::REG_MCP_C1TXATIF), (uint8_t*)&status.tx_attempt_if, 4);
    readAddr(to_underlying(RegisterAddress::REG_MCP_C1TREC), (uint8_t*)&status.trec, 4);
    readAddr(to_underlying(RegisterAddress::REG_MCP_C1BDIAGx), (uint8_t*)&status.bdiag0, 4);
    readAddr(to_underlying(RegisterAddress::REG_MCP_C1BDIAGx) + 4, (uint8_t*)&status.bdiag1, 4);
    readAddr(to_underlying(RegisterAddress::REG_MCP_CRC), (uint8_t*)&status.crc, 4);

    status.bus_off = (status.trec & (1UL << 21)) != 0;
    status.tx_error_passive = (status.trec & (1UL << 20)) != 0;
    status.rx_error_passive = (status.trec & (1UL << 19)) != 0;
    status.tx_error_warning = (status.trec & (1UL << 18)) != 0;
    status.rx_error_warning = (status.trec & (1UL << 17)) != 0;
    status.error_warning = (status.trec & (1UL << 16)) != 0;
    status.tx_error_count = (status.trec >> 8) & 0xFF;
    status.rx_error_count = status.trec & 0xFF;
    status.spi_crc_format_error = (status.crc & (1UL << 17)) != 0;
    status.spi_crc_error = (status.crc & (1UL << 16)) != 0;

    return status;
}

int MCP251863::setControllerMode(ControllerMode contMode) {
    uint16_t addr = to_underlying(RegisterAddress::REG_MCP_C1CON);
    uint8_t buff0, buff1;

    // read current contMode
    readAddr(addr + 2, &buff0, 1);

    if ((buff0 >> 5) != to_underlying(ControllerMode::CMODE_MCP_CONF)) {
        readAddr(addr + 3, &buff1, 1);
        buff1 = (buff1 & 0b11111000) | to_underlying(ControllerMode::CMODE_MCP_CONF);
        writeAddr(addr + 3, &buff1, 1);

        do {
            readAddr(addr + 2, &buff0, 1);
        } while ((buff0 >> 5) != to_underlying(ControllerMode::CMODE_MCP_CONF));
    }

    // write intended contMode
    readAddr(addr + 3, &buff1, 1);
    buff1 = (buff1 & 0b11111000) | to_underlying(contMode);
    writeAddr(addr + 3, &buff1, 1);

    return 1;
}

int MCP251863::setTransceiverMode(TransceiverMode mode) {
    gpio_put(standbyPin_, to_underlying(mode));
    return 1;
}

int MCP251863::setInterrupts(InterruptEnable* intEnArray, size_t intEnSize) {
    // illegal size
    if (intEnSize > 32) {
        return 0;
    }
    uint32_t message = 0;
    for (int i = 0; i < intEnSize; i++) {
        message |= static_cast<uint32_t>(intEnArray[i]);
    }
    writeAddr(to_underlying(RegisterAddress::REG_MCP_C1INT), (uint8_t*)&message, 4);
    return 1;
}

int MCP251863::setPinMode(IoPin pin, IoMode mode) {
    uint8_t buff[4] = {0};
    readAddr(to_underlying(RegisterAddress::REG_MCP_IOCON), buff, 4);
    if (pin == IoPin::IO_MCP_INT0) {
        switch (mode) {
            case IoMode::IOMODE_MCP_GPIO_IN:
                buff[0] |= 0b00000001;
                buff[3] |= 0b00000001;
                break;
            case IoMode::IOMODE_MCP_GPIO_OUT:
                buff[0] &= 0b11111110;
                buff[3] |= 0b00000001;
                break;
            case IoMode::IOMODE_MCP_INT: buff[3] &= 0b11111110; break;
            default:
                return 0;
        }
    } else if (pin == IoPin::IO_MCP_INT1) {
        switch (mode) {
            case IoMode::IOMODE_MCP_GPIO_IN:
                buff[0] |= 0b00000010;
                buff[3] |= 0b00000010;
                break;
            case IoMode::IOMODE_MCP_GPIO_OUT:
                buff[0] &= 0b11111101;
                buff[3] |= 0b00000010;
                break;
            case IoMode::IOMODE_MCP_INT: buff[3] &= 0b11111101; break;
            default:
                return 0;
        }
    } else {
        return 0;
    }
    writeAddr(to_underlying(RegisterAddress::REG_MCP_IOCON), buff, 4);
    return 1;
}

int MCP251863::getTXCode() {
    // C1VEC bits [28:24] = TXCODE, located in byte 3 of the register
    uint8_t buff;
    readAddr(to_underlying(RegisterAddress::REG_MCP_C1VEC) + 3, &buff, 1);
    buff &= 0x1F;
    if (buff > 0b0011111) {
        return -1;
    }
    return buff;
}

int MCP251863::getRXCode() {
    // C1VEC bits [20:16] = RXCODE, located in byte 2 of the register
    uint8_t buff;
    readAddr(to_underlying(RegisterAddress::REG_MCP_C1VEC) + 2, &buff, 1);
    buff &= 0x1F;
    if (buff > 0b0011111) {
        return -1;
    }
    return buff;
}

int MCP251863::getFLTCode() {
    // C1VEC bits [12:8] = FILHIT, located in byte 1 of the register
    uint8_t buff;
    readAddr(to_underlying(RegisterAddress::REG_MCP_C1VEC) + 1, &buff, 1);
    return buff & 0x1F;
}

int MCP251863::getICode() {
    // C1VEC bits [6:0] = ICODE, located in byte 0 of the register
    uint8_t buff;
    readAddr(to_underlying(RegisterAddress::REG_MCP_C1VEC), &buff, 1);
    buff &= 0x7F;
    if (buff > 0b1001010) {
        return -1;
    }
    return buff;
}
