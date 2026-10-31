#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace tap_hw::luckfox_gateway {

inline constexpr std::uint16_t kProtocolVersion = 1U;
inline constexpr std::uint16_t kUdpPort = 8025U;
inline constexpr std::uint32_t kHelloMagic = 0x3147464CU;      // "LFG1"
inline constexpr std::uint32_t kTelemetryMagic = 0x3154474CU;  // "LGT1"
inline constexpr std::uint32_t kActionMagic = 0x3141474CU;     // "LGA1"
inline constexpr std::uint16_t kKindHello = 1U;

// Core-frame order is [LH, LK, LW, RH, RK, RW]. IMU vectors use the body frame
// (+X forward, +Y left, +Z up). Positions are rad, rates rad/s, accel m/s^2,
// gyro rad/s. These are sensor/estimator outputs, not normalized policy inputs.
struct SensorSnapshot {
  std::uint64_t timestamp_us = 0U;
  std::uint32_t sequence = 0U;
  std::array<float, 3> accel_body_m_s2{};
  std::array<float, 3> gyro_body_rad_s{};
  std::array<float, 6> joint_position_rad{};
  std::array<float, 6> joint_velocity_rad_s{};
  float pitch_rad = 0.0F;
  float pitch_rate_rad_s = 0.0F;
  std::uint8_t valid_mask = 0U;  // bit 0 IMU, bit 1 four leg encoders, bit 2 two wheel encoders
};

struct Action {
  std::uint32_t sequence = 0U;
  std::uint64_t observation_timestamp_us = 0U;
  // Legacy UDP draft, not HCR1: raw Beni512 actor order is
  // [a_hipL,a_hipR,a_crankL,a_crankR,a_wheelL,a_wheelR], not decoded targets.
  // The legacy normalized name/clipping must not be copied into HCR1.
  // The gateway records but does not apply these values to motor drivers.
  std::array<float, 6> normalized{};
};

#pragma pack(push, 1)
struct HelloPacket {
  std::uint32_t magic = kHelloMagic;
  std::uint16_t version = kProtocolVersion;
  std::uint16_t kind = kKindHello;
  std::uint32_t client_nonce = 0U;
  std::uint32_t crc32 = 0U;
};

struct TelemetryPacket {
  std::uint32_t magic = kTelemetryMagic;
  std::uint16_t version = kProtocolVersion;
  std::uint16_t payload_size = 0U;
  std::uint32_t sequence = 0U;
  std::uint64_t timestamp_us = 0U;
  std::uint8_t valid_mask = 0U;
  std::uint8_t reserved[3]{};
  float accel_body_m_s2[3]{};
  float gyro_body_rad_s[3]{};
  float joint_position_rad[6]{};
  float joint_velocity_rad_s[6]{};
  float pitch_rad = 0.0F;
  float pitch_rate_rad_s = 0.0F;
  std::uint32_t crc32 = 0U;
};

struct ActionPacket {
  std::uint32_t magic = kActionMagic;
  std::uint16_t version = kProtocolVersion;
  std::uint16_t payload_size = 0U;
  std::uint32_t sequence = 0U;
  std::uint64_t observation_timestamp_us = 0U;
  float normalized[6]{};
  std::uint32_t crc32 = 0U;
};
#pragma pack(pop)

[[nodiscard]] std::uint32_t crc32(const void* data, std::size_t size) noexcept;
[[nodiscard]] bool decodeHello(const std::uint8_t* bytes, std::size_t size,
                               HelloPacket& out) noexcept;
[[nodiscard]] bool decodeAction(const std::uint8_t* bytes, std::size_t size,
                                Action& out) noexcept;
void encodeTelemetry(const SensorSnapshot& snapshot, TelemetryPacket& out) noexcept;

}  // namespace tap_hw::luckfox_gateway
