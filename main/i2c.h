#ifndef I2C_H
#define I2C_H

#define SLAVE_ADDR 0x45

extern void init_i2c();
extern int i2c_read_command(uint8_t in[], size_t len);
extern int i2c_send_response(uint8_t out[], size_t len);

#endif /* I2C_H */
