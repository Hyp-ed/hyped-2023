#include <optional>

#include <io/hardware_gpio.hpp>

#include <core/logger.hpp>

namespace hyped::sensors {
    class Sensor{
        public:
            Sensor(const std::uint8_t newPin);
            ~Sensor();

            int getStripeCount();
            void updateStripes();
        private:
            std::uint8_t pin;
            int stripeCount = 0;
            hyped::core::ILogger &log_;
    };
}
