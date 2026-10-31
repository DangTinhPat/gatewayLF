#pragma once
#ifdef ARDUINO
#include "tap_hw/BlMotor.hpp"
#include <driver/uart.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace tap_hw::hybrid {
// Passive GPIO17 pulse capture, independent of UART's RX FIFO. No RMT TX.
struct MotorPadWave {
  bool available=false;
  std::uint32_t serial=0, status=0, interrupts=0, tick_hz=0;
  std::uint8_t id=0, instruction=0, address=0, error=0, tx_count=0, echo_count=0;
  std::array<std::uint8_t,48> tx{}, echo{};
  std::array<std::uint32_t,96> items{};
};
struct MotorPadWaves { bool ready=false; MotorPadWave good{}, failed{}; };
class EspMotorCritical final : public MotorCritical {
 public:
  void enter() noexcept override { portENTER_CRITICAL(&mux_); }
  void leave() noexcept override { portEXIT_CRITICAL(&mux_); }
 private:
  portMUX_TYPE mux_=portMUX_INITIALIZER_UNLOCKED;
};
class EspMotorBus final : public MotorBus {
 public:
  bool begin(uart_port_t uart, gpio_num_t data, gpio_num_t unused_rx) noexcept;
  std::uint64_t now() const noexcept override;
  BusError lastError() const noexcept override { return error_; }
  BusTrace trace() const noexcept override { return trace_; }
  MotorPadWaves padWaves() noexcept;
  bool request(BusFormat, std::uint8_t, std::uint8_t, const std::uint8_t*,
               std::size_t, std::size_t, BusReply&) noexcept override;
 private:
  bool readExact(std::uint8_t*, std::size_t, std::uint64_t deadline, std::uint8_t& received) noexcept;
  void release() noexcept;
  bool beginPadCapture() noexcept;
  void preparePadCapture() noexcept;
  void startPadCapture() noexcept;
  void finishPadCapture(std::uint8_t, std::uint8_t, std::uint8_t) noexcept;
  uart_port_t uart_=UART_NUM_1;
  gpio_num_t pin_=GPIO_NUM_17;
  std::uint32_t tx_signal_=0;
  bool initialized_=false;
  BusError error_=BusError::None;
  BusTrace trace_{};
  QueueHandle_t events_=nullptr;
  portMUX_TYPE tx_mux_=portMUX_INITIALIZER_UNLOCKED;
  portMUX_TYPE wave_mux_=portMUX_INITIALIZER_UNLOCKED;
  MotorPadWaves waves_{};
  std::uint32_t wave_serial_=0, wave_tick_hz_=0;
};
class EspBlMotor {
 public:
  EspBlMotor() : backend(lock), left(backend,0,left_bus), right(backend,1,right_bus) {}
  bool begin() noexcept;
  MotorPadWaves padWaves() noexcept { return right_bus.padWaves(); }
  EspMotorCritical lock;
  BlMotorBackend backend;
 private:
  static void workerTask(void* arg);
  EspMotorBus left_bus, right_bus;
  BlMotorWorker left, right;
  TaskHandle_t left_task=nullptr, right_task=nullptr;
};
} // namespace tap_hw::hybrid
#endif
