#pragma once

#include <core/logger.hpp>
#include <core/types.hpp>
#include <io/can.hpp>

namespace hyped::sensors {
// Constants defined in IMD CAN reference manual
constexpr std::uint8_t kRequestDataCanId            = 0xA100101;
constexpr std::uint8_t kReturnDataCanId             = 0xA100100;
constexpr std::uint8_t kRequestIsolationResistances = 0xE1;
constexpr std::uint8_t kRequestIsolationState       = 0xE0;

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

  uint16_t getRp();
  uint16_t getRn();
  uint8_t getIsolationState();

 private:
  core::ILogger &logger_;
  std::shared_ptr<io::ICan> can_;
  uint16_t rp_;
  uint16_t rn_;
  uint8_t iso_state_;
};
}  // namespace hyped::sensors