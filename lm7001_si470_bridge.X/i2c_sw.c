#include <stdint.h>
#include <xc.h>
#include "hw.h"
#include "i2c.h"

static void i2c_sendbyte(uint8_t b, I2C_ACK ack) {
    for (int i = 0; i < 8; i++, b <<= 1)  {
        SI_SDA_PORT = (b & 0x80) ? 1 : 0;
        NOP();
        SI_SCK_PORT = 1;
        NOP();
        SI_SCK_PORT = 0;
    }

    // send ACK
    SI_SDA_PORT = ack;
    SI_SCK_PORT = 1;
    NOP();
    SI_SCK_PORT = 0;
    NOP();
    SI_SCK_PORT = 1;
    // Bus released or not depending on ack
}

static uint8_t i2c_rcvbyte(I2C_ACK ack) {
    CLRWDT();
    uint8_t ret = 0;
    for (int i = 0; i < 8; i++, ret <<= 1)  {
        if (SI_SDA_PORT) ret |= 1;
        NOP();
        SI_SCK_PORT = 1;
        NOP();
        SI_SCK_PORT = 0;
    }

    // send ACK
    SI_SDA_PORT = ack;
    SI_SCK_PORT = 1;
    NOP();
    SI_SCK_PORT = 0;
    NOP();
    SI_SCK_PORT = 1;
    // Bus released or not depending on ack
    return ret;
}

void i2c_begin(uint8_t address) {
    // Start bit
    SI_SDA_PORT = 0;
    NOP();
    SI_SCK_PORT = 0;
    
    // Send address
    i2c_sendbyte(address, I2C_ACK_CONTINUE);    
}

void i2c_write_16(uint16_t data, I2C_ACK ack) {
    i2c_sendbyte((uint8_t)(data >> 8), I2C_ACK_CONTINUE);    
    i2c_sendbyte((uint8_t)(data & 0xff), ack);        
}

uint16_t i2c_read_16(I2C_ACK ack) {
    uint16_t ret = i2c_rcvbyte(I2C_ACK_CONTINUE) << 8;
    ret |= i2c_rcvbyte(ack);    
    return ret;
}

