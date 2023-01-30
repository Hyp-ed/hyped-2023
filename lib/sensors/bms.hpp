#pragma once

#include <array>
#include <optional>

#include <bms_processor.hpp>
#include <core/logger.hpp>
#include <io/hardware_can.hpp>

namespace hyped::sensors {

// TODOLater: Confirm with propulsion
static constexpr std::uint8_t NUM_BMS_CELLS               = 12;
static constexpr std::uint8_t NUM_BMS_TEMPERATURE_SENSORS = 4;

// PackeT IDs for BMS CAN messages (taken from ENNOID's GitHub)
// TODOLater: Figure out which of these we actually need and if these are correct
enum class BmsPacketIds {
  kEscSetDuty = 0,
  kEscSetCurrent,
  kEscSetCurrentBrake,
  kEscSetRpm,
  kEscSetPos,
  kFillRxBuffer,
  kFillRxBufferLong,
  kProcessRxBuffer,
  kProcessShortBuffer,
  kEscStatus,
  kEscSetCurrentRel,
  kEscSetCurrentBrakeRel,
  kBmsStatusMainIV = 30,
  kBmsStatusCellVoltage,
  kBmsStatusThrottleChDischBool,
  kBmsStatusTemperatures,
  kBmsStatusAuxIVSafetyWatchdog,
  kBmsKeepAliveSaety,
  kBmsStatusTemperatureIndividual,
  kSlsStatusCurrentRpm = 40,
  kSlsStatusTemperature,
  kSsrStatusMainVTemperature = 60,
  kSsrStatusMainLoad0,
  kSsrStatusMainLoad1
};

enum class BmsCellStates {
  kErrorHardCellVoltage = 0,
  kErrorSoftCellVoltage,
  kErrorOverCurrent,
  kNormal,
};

typedef struct {
  core::Float cellVoltage;
  std::uint8_t cellNumber;
} BmsCell;

typedef struct {
  std::uint8_t throttleDutyCharge;
  std::uint8_t throttleDutyDischarge;
  core::Float SoC;
  core::Float SoCCapacityAh;
  core::Float packVoltage;
  core::Float packCurrent;
  core::Float packPower;
  core::Float loCurrentLoadCurrent;
  core::Float loCurrentLoadVoltage;
  core::Float cellVoltageHigh;
  core::Float cellVoltageLow;
  core::Float cellVoltageAverage;
  core::Float cellVoltageMisMatch;
  std::uint16_t cellBalanceResistorEnableMask;
  std::array<core::Float, NUM_BMS_CELLS> temperatures;
  core::Float tempBatteryHigh;
  core::Float tempBatteryLow;
  core::Float tempBatteryAverage;
  core::Float tempBMSHigh;
  core::Float tempBMSLow;
  core::Float tempBMSAverage;
  std::uint8_t preChargeDesired;
  std::uint8_t disChargeDesired;
  std::uint8_t disChargeLCAllowed;
  std::uint8_t disChargeHCAllowed;
  std::uint8_t chargeDesired;
  std::uint8_t chargeAllowed;
  std::uint8_t safetyOverCANHCSafeNSafe;
  std::uint8_t chargeCurrentDetected;
  std::uint8_t chargeBalanceActive;
  std::uint8_t powerButtonActuated;
  std::uint8_t packInSOA;
  std::uint8_t watchDogTime;
  BmsCell cellVoltagesIndividual[NUM_BMS_CELLS];
  BmsCellStates packOperationalCellState;

} BmsData;

/**
 * @brief Bms class to interface with the ENNOID BMS Gen 1
 * @details This is responsible for receiving voltage and current data from the BMS
 * It also receives the temperature data from the peripheral temperature sensors attached
 */
class Bms {
 public:
  Bms(core::ILogger &logger, io::HardwareCan &can);
  ~Bms() = default;

  std::optional<BmsData> receiveBmsData();

 private:
  std::optional<std::array<core::Float, NUM_BMS_CELLS>> receiveCellTemperature();
  std::optional<io::CanFrame> receiveCellVoltage();
  std::optional<io::CanFrame> receivePackCurrent();

 private:
  core::ILogger &logger_;
  io::HardwareCan &can_;
};

}  // namespace hyped::sensors