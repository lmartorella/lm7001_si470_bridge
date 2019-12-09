typedef enum {
    I2C_ACK_CONTINUE = 0,
    I2C_ACK_STOP = 1
} I2C_ACK;

void i2c_begin(uint8_t address);
void i2c_write_16(uint16_t data, I2C_ACK ack);
uint16_t i2c_read_16(I2C_ACK ack);
