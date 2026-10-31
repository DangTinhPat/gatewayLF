/**
 * @file joint_bus.c
 * @brief Frame building, echo check and reply parsing for the single-wire bus.
 */
#include "joint_bus.h"

#include <stdbool.h>
#include <string.h>

#include "esp_check.h"
#include "esp_timer.h"
#include "esp_idf_version.h"
#if ESP_IDF_VERSION_MAJOR < 5
// ESP32-S3: UART_SCLK_DEFAULT in IDF 5 is the APB clock used by IDF 4.
#define UART_SCLK_DEFAULT UART_SCLK_APB
#endif
#include "esp_rom_gpio.h"
#include "freertos/FreeRTOS.h"
#include "hal/gpio_ll.h"
#include "soc/gpio_sig_map.h"
#include "soc/uart_periph.h"

static const char *TAG = "joint_bus";

#define BROADCAST_ID 0xFE
#define INST_PING 0x01
#define INST_READ 0x02
#define INST_WRITE 0x03

#define MAX_FRAME 48
#define RX_RING 512
#define TX_DONE_TIMEOUT pdMS_TO_TICKS(5)
// Per-read wait. A joint answers ~500 us (register 0x05) after our last byte;
// the BL4012 wheel answers later and gets a longer window. The echo of our own
// frame always uses the short one.
#define RX_TIMEOUT_JOINT pdMS_TO_TICKS(2)
#define RX_TIMEOUT_WHEEL pdMS_TO_TICKS(6)

static uint8_t checksum(const uint8_t *p, size_t len) {
  uint8_t sum = 0;
  for (size_t i = 0; i < len; i++) {
    sum += p[i];
  }
  return (uint8_t)~sum;
}

static void bus_drive(joint_bus_t *bus) {
  esp_rom_gpio_connect_out_signal(bus->pin, bus->tx_signal, false, false);
}

static void bus_release(joint_bus_t *bus) {
  esp_rom_gpio_connect_out_signal(bus->pin, SIG_GPIO_OUT_IDX, false, false);
  gpio_ll_output_disable(&GPIO, bus->pin);
}

esp_err_t joint_bus_init(joint_bus_t *bus, uart_port_t uart, gpio_num_t pin, int baud) {
  memset(bus, 0, sizeof(*bus));
  bus->uart = uart;
  bus->pin = pin;
  bus->tx_signal = UART_PERIPH_SIGNAL(uart, SOC_UART_TX_PIN_IDX);

  const uart_config_t cfg = {
      .baud_rate = baud,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_DEFAULT,
  };
  ESP_RETURN_ON_ERROR(uart_driver_install(uart, RX_RING, 0, 0, NULL, 0), TAG, "driver install");
  ESP_RETURN_ON_ERROR(uart_param_config(uart, &cfg), TAG, "param config");
  ESP_RETURN_ON_ERROR(uart_set_pin(uart, pin, pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE), TAG,
                      "set pin");
  // gpio_set_direction() detaches UART TX from the pad; bus_drive() reattaches
  // it for every frame and bus_release() lets the motor answer.
  ESP_RETURN_ON_ERROR(gpio_set_direction(pin, GPIO_MODE_INPUT_OUTPUT), TAG, "direction");
  ESP_RETURN_ON_ERROR(gpio_set_level(pin, 1), TAG, "level");
  ESP_RETURN_ON_ERROR(gpio_set_pull_mode(pin, GPIO_PULLUP_ONLY), TAG, "pull");
  bus_release(bus);
  ESP_RETURN_ON_ERROR(uart_set_rx_timeout(uart, 2), TAG, "rx timeout");
  return ESP_OK;
}

// Send one frame, check that the bus carried it intact (our echo), then read
// exactly rx_len reply bytes. rx_len == 0 means no reply is expected.
static esp_err_t transact(joint_bus_t *bus, const uint8_t *tx, size_t tx_len, uint8_t *rx,
                          size_t rx_len, TickType_t rx_timeout) {
  uint8_t echo[MAX_FRAME];
  const int64_t started = esp_timer_get_time();
  bus->trace.tx_count = (uint8_t)tx_len;
  bus->trace.expected = (uint8_t)rx_len;
  memcpy(bus->trace.tx, tx, tx_len);

  uart_flush_input(bus->uart);
  bus_drive(bus);
  const int written = uart_write_bytes(bus->uart, tx, tx_len);
  esp_err_t err = uart_wait_tx_done(bus->uart, TX_DONE_TIMEOUT);
  bus_release(bus);
  bus->trace.tx_us = bus->trace.total_us = (uint32_t)(esp_timer_get_time() - started);
  if (err != ESP_OK || written != (int)tx_len) {
    bus->last_error = JB_ERROR_TX;
    bus->err_timeout++;
    return ESP_ERR_TIMEOUT;
  }

  const int echoed = uart_read_bytes(bus->uart, echo, tx_len, RX_TIMEOUT_JOINT);
  bus->trace.echo_count = echoed > 0 ? (uint8_t)echoed : 0;
  memcpy(bus->trace.echo, echo, bus->trace.echo_count);
  bus->trace.echo_us = bus->trace.total_us = (uint32_t)(esp_timer_get_time() - started);
  if (echoed != (int)tx_len || memcmp(echo, tx, tx_len) != 0) {
    bus->last_error = echoed != (int)tx_len ? JB_ERROR_ECHO_TIMEOUT : JB_ERROR_ECHO_MISMATCH;
    bus->err_echo++;
    return ESP_ERR_INVALID_RESPONSE;
  }
  if (rx_len == 0) {
    return ESP_OK;
  }
  const int received = uart_read_bytes(bus->uart, rx, rx_len, rx_timeout);
  bus->trace.rx_count = received > 0 ? (uint8_t)received : 0;
  memcpy(bus->trace.rx, rx, bus->trace.rx_count);
  bus->trace.total_us = (uint32_t)(esp_timer_get_time() - started);
  if (received != (int)rx_len) {
    bus->last_error = JB_ERROR_REPLY_TIMEOUT;
    bus->err_timeout++;
    return ESP_ERR_TIMEOUT;
  }
  return ESP_OK;
}

static esp_err_t request(joint_bus_t *bus, jb_format_t fmt, uint8_t id, uint8_t inst,
                         const uint8_t *params, uint8_t n_params, uint8_t *data, uint8_t data_len,
                         joint_reply_t *reply) {
  bus->last_error = JB_ERROR_NONE;
  memset(&bus->trace, 0, sizeof(bus->trace));
  uint8_t tx[MAX_FRAME];
  uint8_t rx[MAX_FRAME];
  const bool wheel = fmt == JB_FMT_WHEEL;
  const size_t echo_at = wheel ? 5 : 7;  // PRE_LEN PRE_INST PRE_P1
  const size_t tx_len = (size_t)n_params + 6;
  const size_t rx_len = id == BROADCAST_ID ? 0 : (size_t)data_len + echo_at + 4;
  if (tx_len > sizeof(tx) || rx_len > sizeof(rx)) {
    bus->last_error = JB_ERROR_PACKET;
    return ESP_ERR_INVALID_SIZE;
  }

  tx[0] = 0xFF;
  tx[1] = 0xFF;
  tx[2] = id;
  tx[3] = n_params + 2;
  tx[4] = inst;
  if (n_params) {
    memcpy(&tx[5], params, n_params);
  }
  tx[tx_len - 1] = checksum(&tx[2], tx_len - 3);

  esp_err_t err =
      transact(bus, tx, tx_len, rx, rx_len, wheel ? RX_TIMEOUT_WHEEL : RX_TIMEOUT_JOINT);
  if (err != ESP_OK || rx_len == 0) {
    return err;
  }

  const uint8_t pre_p1 = (inst == INST_READ || inst == INST_WRITE) ? params[0] : 0x00;
  if (rx[0] != 0xFF || rx[1] != 0xFF || rx[2] != id || rx[3] != rx_len - 4 ||
      rx[rx_len - 1] != checksum(&rx[2], rx_len - 3) || rx[echo_at] != tx[3] ||
      rx[echo_at + 1] != inst || rx[echo_at + 2] != pre_p1) {
    bus->last_error = JB_ERROR_REPLY_INVALID;
    bus->err_frame++;
    return ESP_ERR_INVALID_RESPONSE;
  }

  if (reply) {
    reply->status = rx[4];
    reply->pos = wheel ? 0 : (int16_t)(rx[5] | (rx[6] << 8));
  }
  if (data_len) {
    memcpy(data, &rx[echo_at + 3], data_len);
  }
  return ESP_OK;
}

esp_err_t bus_ping(joint_bus_t *bus, jb_format_t fmt, uint8_t id, joint_reply_t *reply) {
  return request(bus, fmt, id, INST_PING, NULL, 0, NULL, 0, reply);
}

esp_err_t bus_write(joint_bus_t *bus, jb_format_t fmt, uint8_t id, uint8_t addr,
                    const uint8_t *data, uint8_t len, joint_reply_t *reply) {
  uint8_t params[MAX_FRAME];
  if ((size_t)len + 1 > sizeof(params)) {
    return ESP_ERR_INVALID_SIZE;
  }
  params[0] = addr;
  memcpy(&params[1], data, len);
  return request(bus, fmt, id, INST_WRITE, params, len + 1, NULL, 0, reply);
}

esp_err_t bus_read(joint_bus_t *bus, jb_format_t fmt, uint8_t id, uint8_t addr, uint8_t len,
                   uint8_t *out, joint_reply_t *reply) {
  if (id == BROADCAST_ID) {
    return ESP_ERR_INVALID_ARG;
  }
  const uint8_t params[2] = {addr, len};
  return request(bus, fmt, id, INST_READ, params, 2, out, len, reply);
}

esp_err_t bus_write_mit(joint_bus_t *bus, uint8_t id, const joint_mit_cmd_t *cmd,
                        joint_reply_t *reply) {
  const uint8_t block[10] = {
      (uint8_t)cmd->kp,  (uint8_t)(cmd->kp >> 8),
      (uint8_t)cmd->kd,  (uint8_t)(cmd->kd >> 8),
      (uint8_t)cmd->pos, (uint8_t)((uint16_t)cmd->pos >> 8),
      (uint8_t)cmd->rpm, (uint8_t)((uint16_t)cmd->rpm >> 8),
      (uint8_t)cmd->tau, (uint8_t)((uint16_t)cmd->tau >> 8),
  };
  return bus_write(bus, JB_FMT_JOINT, id, JB_REG_KP, block, sizeof(block), reply);
}
