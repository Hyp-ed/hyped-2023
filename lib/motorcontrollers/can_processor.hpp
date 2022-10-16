#pragma once

#include <cstdint>
#include <core/types.hpp>

namespace hyped::motorcontrollers {

class CanProcessor
{
public:
    CanProcessor();

    bool sendMessage();
    void processMessage(core::CanFrame frame);
}; 
}