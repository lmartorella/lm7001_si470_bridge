#include <stdint.h>
#include <xc.h>
#include "hw.h"
#include "i2c.h"

// Returns 1 if OK
static I2C_ACK_ENUM i2c_sendbyte(uint8_t b) {
    CLRWDT();
    for (int i = 0; i < 8; i++, b <<= 1)  {
        SI_SDA_PORT = (b & 0x80) ? 1 : 0;
        NOP();
        SI_SCK_PORT = 1;
        NOP();
        SI_SCK_PORT = 0; // 400kHz I2C
    }

    // now check ACK
    SI_SDA_TRIS = 1;
    SI_SCK_PORT = 1;
    NOP();
    I2C_ACK_ENUM ack = SI_SDA_PORT;
    SI_SCK_PORT = 0;
    SI_SDA_TRIS = 0;
    SI_SDA_PORT = 1;
    return ack;
}

static uint8_t i2c_rcvbyte(I2C_ACK_ENUM ack) {
    CLRWDT();
    uint8_t ret = 0;
    SI_SDA_TRIS = 1;
    for (int i = 0; i < 8; i++)  {
        ret <<= 1;
        if (SI_SDA_PORT) ret |= 1;
        NOP();
        SI_SCK_PORT = 1;
        NOP();
        SI_SCK_PORT = 0;
    }

    SI_SDA_TRIS = 0;
    NOP();
    // send ACK
    SI_SDA_PORT = ack;
    NOP();
    SI_SCK_PORT = 1;
    NOP();
    SI_SCK_PORT = 0;
    NOP();
    // Restore SDA low to allow stop bit
    SI_SDA_PORT = 0;
    return ret;
}

I2C_ACK_ENUM i2c_start(uint8_t address) {
    // Start bit
    SI_SDA_PORT = 0;
    NOP();
    SI_SCK_PORT = 0;
    
    return i2c_sendbyte(address);    
}

void i2c_stop() {
    SI_SCK_PORT = 1;
    NOP();
    SI_SDA_PORT = 1;
}

I2C_ACK_ENUM i2c_write_16(uint16_t data) {
    if (i2c_sendbyte((uint8_t)(data >> 8)) == I2C_NACK) {   
        return I2C_NACK;
    }
    return i2c_sendbyte((uint8_t)(data & 0xff));        
}

uint16_t i2c_read_16(I2C_ACK_ENUM ack) {
    uint16_t ret = (((uint16_t)i2c_rcvbyte(I2C_ACK)) << 8);
    ret |= i2c_rcvbyte(ack);
    return ret;
}

void i2c_init() {
    SI_SCK_TRIS = SI_SDA_TRIS = 0;
    NOP();
    SI_SCK_PORT = SI_SDA_PORT = 1; 
}