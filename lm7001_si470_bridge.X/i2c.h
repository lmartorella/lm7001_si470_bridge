typedef enum {
    I2C_ACK = 0,
    I2C_NACK = 1
} I2C_ACK_ENUM;

void i2c_init();
I2C_ACK_ENUM i2c_start(uint8_t address);
void i2c_stop();
I2C_ACK_ENUM i2c_write_16(uint16_t data);
uint16_t i2c_read_16(I2C_ACK_ENUM ack);
