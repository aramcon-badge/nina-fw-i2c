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

#ifndef I2C_H
#define I2C_H

#define SLAVE_ADDR 0x45

extern void init_i2c();
extern int i2c_read_command(uint8_t in[], size_t len);
extern int i2c_send_response(uint8_t out[], size_t len);

#endif /* I2C_H */
