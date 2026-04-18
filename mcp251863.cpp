#include "mcp251863.h"

int create_message_obj(
    uint8_t* dst, uint8_t* data,
    msgtype_MCP251863_t msgtype, pl_size_MCP251863_t plSize, uint32_t id, 
    int brsEn
) 
{
    int num_bytes;
    switch (plSize) {
        case PL_SIZE_MCP_0:
            num_bytes = 0;
            break;
        case PL_SIZE_MCP_1:
            num_bytes = 1;
            break;
        case PL_SIZE_MCP_2:
            num_bytes = 2;
            break;
        case PL_SIZE_MCP_3:
            num_bytes = 3;
            break;
        case PL_SIZE_MCP_4:
            num_bytes = 4;
            break;
        case PL_SIZE_MCP_5:
            num_bytes = 5;
            break;
        case PL_SIZE_MCP_6:
            num_bytes = 6;
            break;
        case PL_SIZE_MCP_7:
            num_bytes = 7;
            break;
        case PL_SIZE_MCP_8:
            num_bytes = 8;
            break;
        case PL_SIZE_MCP_12:
            num_bytes = 12;
            break;
        case PL_SIZE_MCP_16:
            num_bytes = 16;
            break;
        case PL_SIZE_MCP_20:
            num_bytes = 20;
            break;
        case PL_SIZE_MCP_24:
            num_bytes = 24;
            break;
        case PL_SIZE_MCP_32:
            num_bytes = 32;
            break;
        case PL_SIZE_MCP_48:
            num_bytes = 48;
            break;
        case PL_SIZE_MCP_64:
            num_bytes = 64;
            break;
        default:
            return 0;
    }
    if (msgtype == CAN_FD_BASE_MCP) {
        //write id
        dst[0] = (uint8_t)(id & 0b11111111);
        dst[1] = (uint8_t)((id >> 8) & 0b111);
        dst[3] = (uint8_t)((id >> 11) & 0b1);
        dst[4] = (uint8_t)(0b10000000) | ((brsEn & 1) << 6) |  plSize;
        dst[5] = 0;
    }
    else {
        return 0;
    }

    for (int i=0; i<num_bytes; i++) {
        dst[8+i] = data[i];
    }

    return 1;
}



MCP251863::MCP251863(spi_inst_t *ispi, uint iCSPin, uint iSTBYPin) {
    spi = ispi;
    CSPin = iCSPin;
    STBYPin = iSTBYPin;
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
    uint16_t addr = REG_MCP_C1FIFOCONx + 4*(fifoNum-1);

    uint8_t intFlags = 0;
    for (int i=0; i<intFlagSize; i++) {
        intFlags |= intFlagArray[i];
    }

    //set first byte, no RTR (CANFD used)
    buff[0] = 0b01000000 | intFlags | (fifoMode << 7);
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

    //write filter address, assumes canSID is a 12 bit address
    buff[0] = 0b11111111 & (canSID >> 1);
    buff[1] = 0b00000000 & (canSID >> 9);
    buff[2] = 0b00000000;
    //0th bit (LSB)
    buff[3] = 0b00000000 & (canSID & 0xFFFE);

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
    uint8_t buff;
    readAddr(REG_MCP_C1VEC, &buff+2, 1);
    if (buff > 0b0011111) {
        return -1;
    }
    return buff;
}

int MCP251863::getRXCode() {
    uint8_t buff;
    readAddr(REG_MCP_C1VEC, &buff+3, 1);
    if (buff > 0b0011111) {
        return -1;
    }
    return buff;
}

int MCP251863::getFLTCode() {
    uint8_t buff;
    readAddr(REG_MCP_C1VEC, &buff+1, 1);
    return buff;
}

int MCP251863::getICode() {
    uint8_t buff;
    readAddr(REG_MCP_C1VEC, &buff+3, 1);
    if (buff > 1001010) {
        return -1;
    }
    return buff;
}
