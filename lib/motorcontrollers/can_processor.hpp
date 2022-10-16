#pragma once

#include <optional>
#include <cstdint>

namespace hyped::motorcontrollers {

class CanProcessor
{
public:
    CanProcessor();

    std::optional<bool> sendMessage();
    std::optional<bool> processMessage();
}; 
}