#include <driver/i2c.h>
#include <soc/i2c_struct.h>
#include <soc/i2c_reg.h>

#include "i2c.h"

static const i2c_port_t i2c_slave_port = I2C_NUM_0;
static intr_handle_t intr_handle;

#define END_CMD (0xEE)

int counter =0;

static void IRAM_ATTR i2c_interrupt_handler(void *arg) {
  counter++;
  ets_printf("I2C Interrupt: %08x, ack_rec=%d, slave_rw=%d, time_out=%d, arb_lost=%d, bus_busy=%d, slave_addressed=%d, byte_trans=%d, reserved7=%d, rx_fifo_cnt=%d, reserved14=%d, tx_fifo_cnt=%d, scl_main_state_last=%d, reserved27=%d, scl_state_last=%d, reserved31=%d!\n", I2C0.int_status.val,
            I2C0.status_reg.ack_rec,
            I2C0.status_reg.slave_rw,
            I2C0.status_reg.time_out,
            I2C0.status_reg.arb_lost,
            I2C0.status_reg.bus_busy,
            I2C0.status_reg.slave_addressed,
            I2C0.status_reg.byte_trans,
            I2C0.status_reg.reserved7,
            I2C0.status_reg.rx_fifo_cnt,
            I2C0.status_reg.reserved14,
            I2C0.status_reg.tx_fifo_cnt,
            I2C0.status_reg.scl_main_state_last,
            I2C0.status_reg.reserved27,
            I2C0.status_reg.scl_state_last,
            I2C0.status_reg.reserved31
  );
}

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
  ets_printf("Interrupt: %d\n",
    i2c_isr_register(i2c_slave_port, i2c_interrupt_handler, nullptr, ESP_INTR_FLAG_SHARED, &intr_handle));
}

int i2c_read_command(uint8_t in[], size_t len) {
  ets_printf("Read, counter: %d\n", counter);
  ets_printf("STATUS ack_rec=%d, slave_rw=%d, time_out=%d, arb_lost=%d, bus_busy=%d, slave_addressed=%d, byte_trans=%d, reserved7=%d, rx_fifo_cnt=%d, reserved14=%d, tx_fifo_cnt=%d, scl_main_state_last=%d, reserved27=%d, scl_state_last=%d, reserved31=%d!\n",
            I2C0.status_reg.ack_rec,
            I2C0.status_reg.slave_rw,
            I2C0.status_reg.time_out,
            I2C0.status_reg.arb_lost,
            I2C0.status_reg.bus_busy,
            I2C0.status_reg.slave_addressed,
            I2C0.status_reg.byte_trans,
            I2C0.status_reg.reserved7,
            I2C0.status_reg.rx_fifo_cnt,
            I2C0.status_reg.reserved14,
            I2C0.status_reg.tx_fifo_cnt,
            I2C0.status_reg.scl_main_state_last,
            I2C0.status_reg.reserved27,
            I2C0.status_reg.scl_state_last,
            I2C0.status_reg.reserved31
  );
  // wait until I2C0.status_reg.bus_busy and slave_rw = 0 (interrupt should flag)
  int bytes_in = 0;
  while (I2C0.status_reg.bus_busy && !I2C0.status_reg.slave_rw && bytes_in < len) {
    bytes_in += i2c_slave_read_buffer(i2c_slave_port, &in[bytes_in], len - bytes_in, 0);
    // wait for interrupt flag
  }
  return bytes_in;
}

int i2c_send_response(uint8_t out[], size_t len) {
  return i2c_slave_write_buffer(i2c_slave_port, out, len, portMAX_DELAY);
}
