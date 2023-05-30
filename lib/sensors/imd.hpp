#pragma once

#include <cstdint>
#include <memory>
#include <optional>

#include <core/logger.hpp>
#include <core/types.hpp>
#include <io/can.hpp>

namespace hyped::sensors {
// Constants defined in IMD CAN reference manual
constexpr std::uint32_t kRequestDataCanId           = 0xA100101;
constexpr std::uint32_t kReturnDataCanId            = 0xA100100;
constexpr std::uint8_t kRequestIsolationResistances = 0xE1;
constexpr std::uint8_t kRequestIsolationState       = 0xE0;

// TODOLater: Test this code with hardware
class Imd : public io::ICanProcessor {
 public:
  static std::optional<std::shared_ptr<Imd>> create(core::ILogger &logger,
                                                    const std::shared_ptr<io::ICan> can);
  Imd(core::ILogger &logger, const std::shared_ptr<io::ICan> can);

  /**
   * @brief Sends "Request Isolation resistances" CAN message to IMD
   */
  core::Result updateValues();

  /**
   * @brief Saves values returned from the IMD
   */
  core::Result processMessage(const io::CanFrame &frame);

  std::uint16_t getResistancePositive();
  std::uint16_t getResistanceNegative();
  std::uint8_t getIsolationStatus();

 private:
  core::ILogger &logger_;
  std::shared_ptr<io::ICan> can_;
  // Referred to as Rp and Rn in the IMD CAN reference manual
  std::uint16_t resistance_positive_;
  std::uint16_t resistance_negative_;
  std::uint8_t isolation_status_;
};
}  // namespace hyped::sensors