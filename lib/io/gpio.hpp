#pragma once

#include "gpio_reader.hpp"
#include "gpio_writer.hpp"


#include <cstdint>
#include <optional>

namespace hyped::io {

//Not dealing with log right now
class Gpio {
 public:
    static constexpr uint8_t kBankNum = 4;
    enum class Direction { kIn = 0, kOut = 1 };


    Gpio();
    uint8_t Read(const uint8_t pin);
    uint8_t Write(const uint8_t ping);
    
    std::optional<GpioReader> getReader(const std::uint8_t pin);
    std::optional<GpioWriter> getWriter(const std::uint8_t pin);
  private:

    static constexpr std::array<off_t, kBankNum> kBases = {0x44e07000, 0x4804c000, 0x481ac000, 0x481ae000};
    static constexpr uint32_t kMmapSize = 0x1000;
    static constexpr uint32_t kData     = 0x138;
    static constexpr uint32_t kClear    = 0x190;
    static constexpr uint32_t kSet      = 0x194;
    Gpio()                              = delete;

    static void initialise();
    static void uninitialise();

    static void *base_mapping_[kBankNum];
    void exportGPIO();
    void attach();
    void setupWait();

    uint32_t pin_;
    Direction direction_;  

    volatile uint32_t *set_;    // set register
    volatile uint32_t *clear_;  // clear register
    volatile uint32_t *data_;   // data register
    uint32_t pin_mask_;         // mask for register access to this pin
    int fd_;                    // file pointer to /sys/class/gpio/gpioXX/value
};

}  // namespace hyped::io