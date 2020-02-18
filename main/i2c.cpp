/*
  I2C Slave Driver for the Arduino NINA Firmware
  Copyright (c) 2020 Uri Shaked

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <driver/i2c.h>
#include <soc/i2c_struct.h>
#include <soc/i2c_reg.h>

#include "i2c.h"

static const i2c_port_t i2c_slave_port = I2C_NUM_0;

void init_i2c() {
  i2c_config_t conf_slave;
  conf_slave.sda_io_num = (gpio_num_t)21;
  conf_slave.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf_slave.scl_io_num = (gpio_num_t)22;
  conf_slave.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf_slave.mode = I2C_MODE_SLAVE;
  conf_slave.slave.addr_10bit_en = 0;
  conf_slave.slave.slave_addr = SLAVE_ADDR;
  ESP_ERROR_CHECK(i2c_param_config(i2c_slave_port, &conf_slave));
  ESP_ERROR_CHECK(i2c_driver_install(i2c_slave_port, conf_slave.mode, 1024, 1024, ESP_INTR_FLAG_SHARED));
}

int i2c_read_command(uint8_t in[], size_t len) {
  int bytes_in = 0;
  uint8_t chunk_len = 0;
  do {
    if (!i2c_slave_read_buffer(i2c_slave_port, &chunk_len, sizeof(chunk_len), portMAX_DELAY)) {
      return bytes_in;
    }
    if (chunk_len > 0) {
      int read_size = chunk_len;
      if (bytes_in + read_size > len) {
        read_size = len - bytes_in;
      }
      int res = i2c_slave_read_buffer(i2c_slave_port, &in[bytes_in], read_size, 100 * portTICK_PERIOD_MS);
      if (res) {
        bytes_in += chunk_len;
      }
    }
  } while (bytes_in < len && chunk_len > 0);
  return bytes_in;
}

int i2c_send_response(uint8_t out[], size_t len) {
  uint8_t preamble = 0;
  i2c_slave_write_buffer(i2c_slave_port, &preamble, 1, 1000 * portTICK_PERIOD_MS);
  return i2c_slave_write_buffer(i2c_slave_port, out, len, 1000 * portTICK_PERIOD_MS);
}
