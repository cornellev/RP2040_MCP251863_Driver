#include "mcp251863.h"

static int canfd_len_to_dlc(size_t len, pl_size_MCP251863_t* dlc) {
    switch (len) {
        case 0:  *dlc = PL_SIZE_MCP_0;  return 1;
        case 1:  *dlc = PL_SIZE_MCP_1;  return 1;
        case 2:  *dlc = PL_SIZE_MCP_2;  return 1;
        case 3:  *dlc = PL_SIZE_MCP_3;  return 1;
        case 4:  *dlc = PL_SIZE_MCP_4;  return 1;
        case 5:  *dlc = PL_SIZE_MCP_5;  return 1;
        case 6:  *dlc = PL_SIZE_MCP_6;  return 1;
        case 7:  *dlc = PL_SIZE_MCP_7;  return 1;
        case 8:  *dlc = PL_SIZE_MCP_8;  return 1;
        case 12: *dlc = PL_SIZE_MCP_12; return 1;
        case 16: *dlc = PL_SIZE_MCP_16; return 1;
        case 20: *dlc = PL_SIZE_MCP_20; return 1;
        case 24: *dlc = PL_SIZE_MCP_24; return 1;
        case 32: *dlc = PL_SIZE_MCP_32; return 1;
        case 48: *dlc = PL_SIZE_MCP_48; return 1;
        case 64: *dlc = PL_SIZE_MCP_64; return 1;
        default: return 0;
    }
}

static uint8_t canfd_dlc_to_len(uint8_t dlc) {
    switch (dlc & 0x0F) {
        case PL_SIZE_MCP_0:  return 0;
        case PL_SIZE_MCP_1:  return 1;
        case PL_SIZE_MCP_2:  return 2;
        case PL_SIZE_MCP_3:  return 3;
        case PL_SIZE_MCP_4:  return 4;
        case PL_SIZE_MCP_5:  return 5;
        case PL_SIZE_MCP_6:  return 6;
        case PL_SIZE_MCP_7:  return 7;
        case PL_SIZE_MCP_8:  return 8;
        case PL_SIZE_MCP_12: return 12;
        case PL_SIZE_MCP_16: return 16;
        case PL_SIZE_MCP_20: return 20;
        case PL_SIZE_MCP_24: return 24;
        case PL_SIZE_MCP_32: return 32;
        case PL_SIZE_MCP_48: return 48;
        case PL_SIZE_MCP_64: return 64;
        default: return 0;
    }
}

static int pl_size_to_num_bytes(pl_size_MCP251863_t plSize) {
    return canfd_dlc_to_len(plSize);
}

static uint8_t can_dlc_to_len(uint8_t dlc, bool fdf) {
    if (!fdf && ((dlc & 0x0F) > 8)) {
        return 8;
    }
    return canfd_dlc_to_len(dlc);
}

static uint32_t encode_bit_timing(bit_timing_MCP251863_t timing) {
    return ((uint32_t)timing.brp << 24) |
        ((uint32_t)timing.tseg1 << 16) |
        ((uint32_t)timing.tseg2 << 8) |
        timing.sjw;
}

static uint32_t encode_tdc(bool enable, uint8_t offset) {
    if (!enable) {
        return 0;
    }

    return (2UL << 16) | offset;
}

static init_config_MCP251863_t default_init_config() {
    init_config_MCP251863_t config;

    config.enablePll = 0;
    config.sclkDiv2 = 0;
    config.enableTdc = 1;
    config.rxTimestampEnable = 0;
    config.tdcOffset = 6;
    config.txFifo = 1;
    config.rxFifo = 2;
    config.txFifoDepth = 7;
    config.rxFifoDepth = 7;
    config.txPayloadSize = PL_SIZE_MCP_64;
    config.rxPayloadSize = PL_SIZE_MCP_64;

    // Defaults assume a 40 MHz CAN clock: nominal 500 kbit/s, data 2 Mbit/s.
    config.nominalBitTiming = MCP251863_BITTIMING_500K_40MHZ;
    config.dataBitTiming = MCP251863_BITTIMING_2M_40MHZ;

    return config;
}

static uint32_t pack_id_word(const canfd_frame_MCP251863_t& frame) {
    if (frame.ide) {
        return ((frame.id >> 18) & 0x7FF) | ((frame.id & 0x3FFFF) << 11);
    }

    uint32_t sid = frame.id & 0x7FF;
    uint32_t sid11 = (frame.sid11 || (frame.id > 0x7FF)) ? ((frame.id >> 11) & 0x01) : 0;
    return sid | (sid11 << 29);
}

static uint32_t pack_control_word(const canfd_frame_MCP251863_t& frame) {
    return ((frame.sequence & 0x7FFFFF) << 9) |
        ((uint32_t)(frame.esi ? 1 : 0) << 8) |
        ((uint32_t)(frame.fdf ? 1 : 0) << 7) |
        ((uint32_t)(frame.brs ? 1 : 0) << 6) |
        ((uint32_t)(frame.rtr ? 1 : 0) << 5) |
        ((uint32_t)(frame.ide ? 1 : 0) << 4) |
        (frame.dlc & 0x0F);
}

static void store_word(uint8_t* dst, uint32_t word) {
    dst[0] = (uint8_t)(word & 0xFF);
    dst[1] = (uint8_t)((word >> 8) & 0xFF);
    dst[2] = (uint8_t)((word >> 16) & 0xFF);
    dst[3] = (uint8_t)((word >> 24) & 0xFF);
}

static uint32_t load_word(const uint8_t* src) {
    return ((uint32_t)src[0]) |
        ((uint32_t)src[1] << 8) |
        ((uint32_t)src[2] << 16) |
        ((uint32_t)src[3] << 24);
}

static int finalize_frame_dlc(canfd_frame_MCP251863_t* frame) {
    if (frame->fdf) {
        pl_size_MCP251863_t dlc;
        if (!canfd_len_to_dlc(frame->len, &dlc)) {
            return 0;
        }
        frame->dlc = dlc;
        return 1;
    }

    if (frame->len > 8) {
        return 0;
    }
    frame->dlc = frame->len;
    frame->brs = 0;
    return 1;
}

static int validate_tx_frame(const canfd_frame_MCP251863_t& frame) {
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

static canfd_frame_MCP251863_t decode_rx_header(const uint8_t* header, bool timestampEnabled) {
    canfd_frame_MCP251863_t frame;
    memset(&frame, 0, sizeof(frame));

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

int create_message_obj(
    uint8_t* dst, const uint8_t* data,
    msgtype_MCP251863_t msgtype, pl_size_MCP251863_t plSize, uint32_t id, 
    int brsEn
) 
{
    int num_bytes = pl_size_to_num_bytes(plSize);
    if ((num_bytes == 0) && (plSize != PL_SIZE_MCP_0)) {
        return 0;
    }
    if ((num_bytes > 0) && (data == NULL)) {
        return 0;
    }

    canfd_frame_MCP251863_t frame;
    memset(&frame, 0, sizeof(frame));
    frame.id = id;
    frame.ide = (msgtype == CAN_EXT) || (msgtype == CAN_FD_EXT);
    frame.fdf = (msgtype == CAN_FD_BASE_MCP) || (msgtype == CAN_FD_EXT);
    frame.brs = brsEn != 0;
    frame.len = num_bytes;
    frame.dlc = plSize;
    frame.valid = 1;

    for (int i=0; i<num_bytes; i++) {
        frame.data[i] = data[i];
    }

    size_t objectSize = 0;
    return create_message_obj(dst, frame, &objectSize);
}

int create_message_obj(uint8_t* dst, const canfd_frame_MCP251863_t& frame, size_t* objectSize) {
    canfd_frame_MCP251863_t txFrame = frame;
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


MCP251863::MCP251863(spi_inst_t *ispi, uint iCSPin, uint iSTBYPin) {
    spi = ispi;
    CSPin = iCSPin;
    STBYPin = iSTBYPin;
    writeMode = WM_MCP_NORM;
    readMode = RM_MCP_NORM;
    txFifoNum = 1;
    rxFifoNum = 2;
    rxTimestampEnabled = 0;
}

int MCP251863::writeAddr(uint16_t startAddr, uint8_t* data, size_t len) {
    cmd_MCP251863_t cmd;
    switch(writeMode) {
        case WM_MCP_NORM:
            cmd = CMD_MCP_WRITA;
            break;
        case WM_MCP_CRC:
            cmd = CMD_MCP_WRACR;
            break;
        case WM_MCP_SAFE:
            cmd = CMD_MCP_WRASF;
        default: 
            return 0;
    } 
    // form message CCCC-AAAAAAAAAAAA
    uint8_t message[2]; 
    
    message[0] = (cmd << 4) | (startAddr >> 8);
    message[1] = (startAddr << 4) >> 4;

    // drive CS pin low
    asm volatile("nop \n nop \n nop");
    gpio_put(CSPin, 0);
    asm volatile("nop \n nop \n nop");

    // transmit message via SPI
    spi_write_blocking(spi, message, 2);

    // write data
    spi_write_blocking(spi, data, len);

    // drive CS pin high, ending read cycle
    asm volatile("nop \n nop \n nop");
    gpio_put(CSPin, 1);
    asm volatile("nop \n nop \n nop");

    return 1;
}

int MCP251863::readAddr(uint16_t startAddr, uint8_t* dst, size_t len) {
    // set read command based on read mode
    cmd_MCP251863_t cmd;
    uint8_t message[3];
    uint16_t crc;
    switch(readMode) {
        case RM_MCP_NORM:
            cmd = CMD_MCP_READA;
            break;
        case RM_MCP_CRC:
            cmd = CMD_MCP_RDACR;
            break;
        default: 
            return 0;
    } 
    // form message CCCC-AAAAAAAAAAAA
    message[0] = (cmd << 4) | (startAddr >> 8);
    message[1] = (startAddr << 4) >> 4;

    //if (readMode == rm_MCP251863_t::READ_CRC) {
    //    message[2] = (uint8_t)len;
    //}

    // drive CS pin low
    asm volatile("nop \n nop \n nop");
    gpio_put(CSPin, 0);
    asm volatile("nop \n nop \n nop");

    // transmit message via SPI
    spi_write_blocking(spi, message, 2);

    // write extra bits for len if CRC mode
    //if (readMode == rm_MCP251863_t::READ_CRC) {
    //    spi_write_blocking(spi, (message)+2, 1);
    //}

    // read out data
    spi_read_blocking(spi, 0, dst, len);
    
    // read out CRC
    //if (readMode == rm_MCP251863_t::READ_CRC) {
    //    spi_read16_blocking(spi, 0, &crc, 1);
    //}

    // drive CS pin high, ending read cycle
    asm volatile("nop \n nop \n nop");
    gpio_put(CSPin, 1);
    asm volatile("nop \n nop \n nop");

    // check CRC
    // later

    return 1;
}

int MCP251863::init() {
    return init(default_init_config());
}

int MCP251863::init(const init_config_MCP251863_t& config) {
    if ((config.txFifo < 1) || (config.txFifo > 31) ||
        (config.rxFifo < 1) || (config.rxFifo > 31) ||
        (config.txFifo == config.rxFifo) ||
        (config.txFifoDepth > 31) || (config.rxFifoDepth > 31)) {
        return 0;
    }

    uint8_t one = 1;
    uint8_t zero = 0;
    uint32_t reg = 0;

    gpio_init(CSPin);
    gpio_set_dir(CSPin, GPIO_OUT);
    gpio_put(CSPin, 1);
    gpio_init(STBYPin);
    gpio_set_dir(STBYPin, GPIO_OUT);
    setTranMode(TMODE_MCP_STBY);

    writeMode = WM_MCP_NORM;
    readMode = RM_MCP_NORM;

    if (!reset()) {
        return 0;
    }
    sleep_ms(10);

    // Wait for oscillator stability before touching CAN timing.
    for (int i=0; i<100; i++) {
        readAddr(REG_MCP_OSC + 1, &one, 1);
        if ((one & (1 << 2)) != 0) {
            break;
        }
        sleep_ms(1);
        if (i == 99) {
            return 0;
        }
    }

    if (!setContMode(CMODE_MCP_CONF)) {
        return 0;
    }

    uint8_t osc = (config.enablePll ? 0x01 : 0x00) | (config.sclkDiv2 ? 0x10 : 0x00);
    writeAddr(REG_MCP_OSC, &osc, 1);
    if (config.enablePll) {
        for (int i=0; i<100; i++) {
            readAddr(REG_MCP_OSC + 1, &one, 1);
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
    writeAddr(REG_MCP_C1TDC, (uint8_t*)&reg, 4);

    txFifoNum = config.txFifo;
    rxFifoNum = config.rxFifo;
    rxTimestampEnabled = config.rxTimestampEnable;

    fifo_int_mode_MCP251863_t txFlags[] = {FIFO_INT_MCP_NFNE, FIFO_INT_MCO_TXAT};
    fifo_int_mode_MCP251863_t rxFlags[] = {FIFO_INT_MCP_NFNE, FIFO_INT_MCP_OVFL};
    if (!initGPFIFO(txFifoNum, FIFO_MODE_MCP_TX, config.txPayloadSize,
        config.txFifoDepth, 1, TXRET_MCP_UNLIM, txFlags, 2)) {
        return 0;
    }
    if (!initGPFIFO(rxFifoNum, FIFO_MODE_MCP_RX, config.rxPayloadSize,
        config.rxFifoDepth, 0, TXRET_MCP_NONE, rxFlags, 2)) {
        return 0;
    }
    if (rxTimestampEnabled) {
        uint16_t rx_fifo_addr = REG_MCP_C1FIFOCONx + 12*(rxFifoNum-1);
        readAddr(rx_fifo_addr, &one, 1);
        one |= (1 << 5);
        writeAddr(rx_fifo_addr, &one, 1);
    }

    // Disable filter 0 while programming it, then make it accept all frames into rxFifoNum.
    uint16_t flt_ctrl_addr = REG_MCP_C1FLTCONx;
    writeAddr(flt_ctrl_addr, &zero, 1);
    reg = 0;
    writeAddr(REG_MCP_C1FLTOBJx, (uint8_t*)&reg, 4);
    writeAddr(REG_MCP_C1MASKx, (uint8_t*)&reg, 4);
    one = 0b10000000 | (rxFifoNum & 0b00011111);
    writeAddr(flt_ctrl_addr, &one, 1);

    reg = 0xFFFFFFFF;
    writeAddr(REG_MCP_C1RXIF, (uint8_t*)&reg, 4);
    writeAddr(REG_MCP_C1TXIF, (uint8_t*)&reg, 4);
    writeAddr(REG_MCP_C1RXOVIF, (uint8_t*)&reg, 4);
    writeAddr(REG_MCP_C1TXATIF, (uint8_t*)&reg, 4);

    int_en_MCP251863_t interrupts[] = {
        INT_EN_MCP_RXIE,
        INT_EN_MCP_TXIE,
        INT_EN_MCP_RXOVIE,
        INT_EN_MCP_TXATIE,
        INT_EN_MCP_CERRIE,
        INT_EN_MCP_SERRIE
    };
    if (!setInterrupts(interrupts, sizeof(interrupts) / sizeof(interrupts[0]))) {
        return 0;
    }

    setTranMode(TMODE_MCP_NORM);
    if (!setContMode(CMODE_MCP_CFD_NORM)) {
        return 0;
    }

    for (int i=0; i<100; i++) {
        readAddr(REG_MCP_C1CON + 2, &one, 1);
        if ((one >> 5) == CMODE_MCP_CFD_NORM) {
            return 1;
        }
        sleep_ms(1);
    }

    return 0;
}

int MCP251863::setBitTiming(bit_timing_MCP251863_t nominalTiming, bit_timing_MCP251863_t dataTiming) {
    uint32_t reg = encode_bit_timing(nominalTiming);
    writeAddr(REG_MCP_C1NBTCFG, (uint8_t*)&reg, 4);

    reg = encode_bit_timing(dataTiming);
    writeAddr(REG_MCP_C1DBTCFG, (uint8_t*)&reg, 4);

    return 1;
}

int MCP251863::reset() {
    cmd_MCP251863_t cmd = CMD_MCP_RESET;
    uint8_t message[2] = {0};

    message[0] = cmd << 4;


    // drive CS pin low
    asm volatile("nop \n nop \n nop");
    gpio_put(CSPin, 0);
    asm volatile("nop \n nop \n nop");

    // transmit message via SPI
    spi_write_blocking(spi, message, 2);

    // drive CS pin high, ending read cycle
    asm volatile("nop \n nop \n nop");
    gpio_put(CSPin, 1);
    asm volatile("nop \n nop \n nop");

    return 1;
}

int MCP251863::initGPFIFO(
    uint8_t fifoNum, fifo_mode_MCP251863_t fifoMode, 
    pl_size_MCP251863_t plSize, uint8_t fSize,
    uint8_t prioNum, tx_retran_mode_MCP251863_t retranMode,
    fifo_int_mode_MCP251863_t* intFlagArray, size_t intFlagSize
)
{
    uint8_t buff[4];
    uint16_t addr = REG_MCP_C1FIFOCONx + 12*(fifoNum-1);

    uint8_t intFlags = 0;
    for (int i=0; i<intFlagSize; i++) {
        intFlags |= intFlagArray[i];
    }

    buff[0] = intFlags | (fifoMode << 7);
    buff[1] = 0b00000000; 
    //assumes prioNum <= 32
    buff[2] = 0b00000000 | (retranMode << 5) | prioNum;
    //assumes fSize <= 32
    buff[3] = ((plSize & 0b111) << 5) | fSize;

    writeAddr(addr, buff, 4);
    return 1;
}

int MCP251863::initTEFIFO(
    uint8_t fSize, 
    fifo_int_mode_MCP251863_t* intFlagArray, size_t intFlagSize
)
{
    uint8_t buff[4];
    reg_addr_MCP251863_t addr = REG_MCP_C1TEFCON;

    uint8_t intFlags = 0;
    for (int i=0; i<intFlagSize; i++) {
        intFlags |= intFlagArray[i];
    }

    //wait for reset bit to clear
    do {
        readAddr(addr + 1, buff, 1);
    } while ((buff[0] >> 2) == 1);

    //set bytes
    buff[0] = 0b00000000 | intFlags;
    buff[1] = 0b00000000;
    buff[2] = 0b00000000;
    //assumes fSize <= 32
    buff[3] = 0b00000000 | fSize;

    writeAddr(addr, buff, 4);
    return 1;
}

int MCP251863::initTXQ(
    pl_size_MCP251863_t plSize, uint8_t fSize,
    uint8_t prioNum, tx_retran_mode_MCP251863_t retranMode,
    fifo_int_mode_MCP251863_t* intFlagArray, size_t intFlagSize
)
{
    uint8_t buff[4];
    reg_addr_MCP251863_t addr = REG_MCP_C1TXQCON ;

    uint8_t intFlags = 0;
    for (int i=0; i<intFlagSize; i++) {
        intFlags |= intFlagArray[i];
    }

    //wait for reset bit to clear
    do {
        readAddr(addr + 1, buff, 1);
    } while ((buff[0] >> 2) == 1);

    //set bytes
    buff[0] = 0b00000000 | intFlags;
    buff[1] = 0b00000000;
    //assumes prioNum <= 32
    buff[2] = 0b00000000 | (retranMode << 5) | prioNum;
    //assumes fSize <= 32
    buff[3] = ((plSize & 0b111) << 5) | fSize;

    writeAddr(addr, buff, 4);
    return 1;
}

int MCP251863::initFilter(uint8_t fltNum, uint8_t fifoNum, uint16_t canSID) {
    uint16_t flt_addr = REG_MCP_C1FLTCONx + fltNum/4;
    uint16_t flt_obj_addr = REG_MCP_C1FLTOBJx + 8*fltNum;

    uint8_t buff[4];

    //enable filter, assumes fifoNum <= 32
    buff[0] = 0b00000000 | fifoNum | (1 << 7);
    writeAddr(flt_addr + fltNum%4, buff, 1);

    // Pack SID[10:0] (and SID11 in bit 11) into C1FLTOBJn bits [11:0]
    buff[0] = canSID & 0xFF;
    buff[1] = (canSID >> 8) & 0x0F;
    buff[2] = 0x00;
    buff[3] = 0x00;

    writeAddr(flt_obj_addr, buff, 4);

    return 1;
}

int MCP251863::pushTXFIFO(uint8_t fifoNum, uint8_t* data, size_t pSize) {
    uint16_t fifo_addr = REG_MCP_C1FIFOCONx + 12*(fifoNum-1);
    uint16_t fifo_stat_addr = REG_MCP_C1FIFOSTAx + 12*(fifoNum-1);
    uint16_t fifo_point_addr = REG_MCP_C1FIFOUAx + 12*(fifoNum-1);

    uint8_t buff;
    uint16_t message_addr;

    //check if fifo nonfull, return if it is. We will implement real error handling later
    readAddr(fifo_stat_addr, &buff, 1);
    if ((buff & 0b00000001) == 0) {
        return 0;
    }

    //get pointer to message object from FIFO, the addresses are only 12-bits wide?
    readAddr(fifo_point_addr, (uint8_t*)&message_addr, 2);
    message_addr += 0x400; 

    //write message to addr
    writeAddr(message_addr, data, pSize);

    //increment pointer
    buff = 0b00000001;
    writeAddr(fifo_addr+1, &buff, 1);

    return 1;
}

int MCP251863::reqSendTXFIFO(uint8_t fifoNum) {
    uint16_t addr = REG_MCP_C1FIFOCONx + 12*(fifoNum-1);
    uint8_t buff;
    
    //dont know if this is needed, but wait until the fifo increments
    do {
        readAddr(addr+1, &buff, 1);
    } while ((buff & 0b00000001) == 1);

    //set bit to request txfifo send
    buff = 0b00000010;
    writeAddr(addr+1, &buff, 1);
    
    return 1;
}

int MCP251863::popRXFIFO(uint8_t fifoNum, uint8_t* dst, size_t pSize) {
    uint16_t fifo_addr = REG_MCP_C1FIFOCONx + 12*(fifoNum-1);
    uint16_t fifo_stat_addr = REG_MCP_C1FIFOSTAx + 12*(fifoNum-1);
    uint16_t fifo_point_addr = REG_MCP_C1FIFOUAx + 12*(fifoNum-1);

    uint8_t buff;
    uint16_t message_addr;

    //check if fifo nonempty, if it is return
    readAddr(fifo_stat_addr, &buff, 1);
    if ((buff & 0b00000001) == 0) {
        return 0;
    }

    readAddr(fifo_point_addr, (uint8_t*)&message_addr, 2);
    message_addr += 0x400;

    //read in message
    readAddr(message_addr, dst, pSize);

    //decrement fifo
    buff = 0b00000001;
    writeAddr(fifo_addr+1, &buff, 1);

    return 1;
}

int MCP251863::send_canfd(uint32_t id, const uint8_t* data, size_t len, bool brs, bool extended_id) {
    return send_canfd(txFifoNum, id, data, len, brs, extended_id);
}

int MCP251863::send_canfd(uint8_t fifoNum, uint32_t id, const uint8_t* data, size_t len, bool brs, bool extended_id) {
    pl_size_MCP251863_t dlc;
    if (!canfd_len_to_dlc(len, &dlc)) {
        return 0;
    }
    if ((len > 0) && (data == NULL)) {
        return 0;
    }

    canfd_frame_MCP251863_t frame;
    memset(&frame, 0, sizeof(frame));
    frame.id = id;
    frame.ide = extended_id;
    frame.fdf = 1;
    frame.brs = brs;
    frame.len = len;
    frame.dlc = dlc;
    for (size_t i=0; i<len; i++) {
        frame.data[i] = data[i];
    }
    return send_frame(fifoNum, frame);
}

int MCP251863::send_frame(const canfd_frame_MCP251863_t& frame) {
    return send_frame(txFifoNum, frame);
}

int MCP251863::send_frame(uint8_t fifoNum, const canfd_frame_MCP251863_t& frame) {
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

canfd_frame_MCP251863_t MCP251863::read_canfd() {
    return read_frame(rxFifoNum);
}

canfd_frame_MCP251863_t MCP251863::read_canfd(uint8_t fifoNum) {
    return read_frame(fifoNum);
}

canfd_frame_MCP251863_t MCP251863::read_frame() {
    return read_frame(rxFifoNum);
}

canfd_frame_MCP251863_t MCP251863::read_frame(uint8_t fifoNum) {
    canfd_frame_MCP251863_t frame;
    memset(&frame, 0, sizeof(frame));

    uint16_t fifo_addr = REG_MCP_C1FIFOCONx + 12*(fifoNum-1);
    uint16_t fifo_stat_addr = REG_MCP_C1FIFOSTAx + 12*(fifoNum-1);
    uint16_t fifo_point_addr = REG_MCP_C1FIFOUAx + 12*(fifoNum-1);

    uint8_t buff;
    uint16_t message_addr;
    uint8_t header[12] = {0};
    size_t headerSize = rxTimestampEnabled ? 12 : 8;

    readAddr(fifo_stat_addr, &buff, 1);
    if ((buff & 0b00000001) == 0) {
        return frame;
    }

    readAddr(fifo_point_addr, (uint8_t*)&message_addr, 2);
    message_addr += 0x400;
    readAddr(message_addr, header, headerSize);
    frame = decode_rx_header(header, rxTimestampEnabled);

    if (frame.len > 0) {
        readAddr(message_addr + headerSize, frame.data, frame.len);
    }

    buff = 0b00000001;
    writeAddr(fifo_addr+1, &buff, 1);

    frame.valid = 1;
    return frame;
}

fifo_status_MCP251863_t MCP251863::getFIFOStatus(uint8_t fifoNum) {
    fifo_status_MCP251863_t status;
    memset(&status, 0, sizeof(status));

    uint16_t fifo_stat_addr = REG_MCP_C1FIFOSTAx + 12*(fifoNum-1);
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

status_MCP251863_t MCP251863::getStatus() {
    status_MCP251863_t status;
    memset(&status, 0, sizeof(status));

    readAddr(REG_MCP_C1INT, (uint8_t*)&status.interrupt_flags, 4);
    readAddr(REG_MCP_C1RXIF, (uint8_t*)&status.rx_if, 4);
    readAddr(REG_MCP_C1TXIF, (uint8_t*)&status.tx_if, 4);
    readAddr(REG_MCP_C1RXOVIF, (uint8_t*)&status.rx_overflow_if, 4);
    readAddr(REG_MCP_C1TXATIF, (uint8_t*)&status.tx_attempt_if, 4);
    readAddr(REG_MCP_C1TREC, (uint8_t*)&status.trec, 4);
    readAddr(REG_MCP_C1BDIAGx, (uint8_t*)&status.bdiag0, 4);
    readAddr(REG_MCP_C1BDIAGx + 4, (uint8_t*)&status.bdiag1, 4);
    readAddr(REG_MCP_CRC, (uint8_t*)&status.crc, 4);

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

int MCP251863::setContMode(cmode_MCP251863_t contMode) {
    uint16_t addr = REG_MCP_C1CON;
    uint8_t buff0, buff1;

    //read current contMode 
    readAddr(addr+2, &buff0, 1);

    if ((buff0 >> 5) != CMODE_MCP_CONF) {
        readAddr(addr+3, &buff1, 1);
        buff1 = (buff1 & 0b11111000) | CMODE_MCP_CONF;
        writeAddr(addr+3, &buff1, 1);

        do {
            readAddr(addr+2, &buff0, 1);
        } while ((buff0 >> 5) != CMODE_MCP_CONF);

    }

    //write intended contMode
    readAddr(addr+3, &buff1, 1);
    buff1 = (buff1 & 0b11111000) | contMode;
    writeAddr(addr+3, &buff1, 1);

    return 1;
}

int MCP251863::setTranMode(tmode_MCP251863_t mode) {
    gpio_put(STBYPin, mode);
    return 1;
}

int MCP251863::setInterrupts(int_en_MCP251863_t* intEnArray, size_t intEnSize) {
    //illegal size
    if (intEnSize > 32) {
        return 0;
    }
    uint32_t message = 0;
    for (int i=0; i<intEnSize; i++) {
        message |= intEnArray[i];
    }
    writeAddr(REG_MCP_C1INT, (uint8_t*)&message, 4);
    return 1;
}

int MCP251863::setPinMode(io_num_MCP251863_t pin, iomode_MCP251863_t mode) {
    uint8_t buff[4]= {0};
    readAddr(REG_MCP_IOCON, buff, 4);
    if (pin == IO_MCP_INT0) {
        switch (mode) {
            case IOMODE_MCP_GPIO_IN:
                buff[0] |= 0b00000001;
                buff[3] |= 0b00000001;
                break;
            case IOMODE_MCP_GPIO_OUT:
                buff[0] &= 0b11111110;
                buff[3] |= 0b00000001;
                break;
            case IOMODE_MCP_INT:
                buff[3] &= 0b11111110;
                break;
            default:
                return 0;
        }
    } 
    else if (pin == IO_MCP_INT1) {
        switch (mode) {
            case IOMODE_MCP_GPIO_IN:
                buff[0] |= 0b00000010;
                buff[3] |= 0b00000010;
                break;
            case IOMODE_MCP_GPIO_OUT:
                buff[0] &= 0b11111101;
                buff[3] |= 0b00000010;
                break;
            case IOMODE_MCP_INT:
                buff[3] &= 0b11111101;
                break;
            default:
                return 0;
        }
    }
    else {
        return 0;
    }
    writeAddr(REG_MCP_IOCON, buff, 4);
    return 1;
}

int MCP251863::getTXCode() {
    // C1VEC bits [28:24] = TXCODE, located in byte 3 of the register
    uint8_t buff;
    readAddr(REG_MCP_C1VEC + 3, &buff, 1);
    buff &= 0x1F;
    if (buff > 0b0011111) {
        return -1;
    }
    return buff;
}

int MCP251863::getRXCode() {
    // C1VEC bits [20:16] = RXCODE, located in byte 2 of the register
    uint8_t buff;
    readAddr(REG_MCP_C1VEC + 2, &buff, 1);
    buff &= 0x1F;
    if (buff > 0b0011111) {
        return -1;
    }
    return buff;
}

int MCP251863::getFLTCode() {
    // C1VEC bits [12:8] = FILHIT, located in byte 1 of the register
    uint8_t buff;
    readAddr(REG_MCP_C1VEC + 1, &buff, 1);
    return buff & 0x1F;
}

int MCP251863::getICode() {
    // C1VEC bits [6:0] = ICODE, located in byte 0 of the register
    uint8_t buff;
    readAddr(REG_MCP_C1VEC, &buff, 1);
    buff &= 0x7F;
    if (buff > 0b1001010) {
        return -1;
    }
    return buff;
}
