#pragma once
#include <Arduino.h>
#include "node_shared.hpp"
namespace lora {

    bool performJoin(uint8_t &assignedNodeID, uint32_t deviceID, int8_t maxRetries);

}
