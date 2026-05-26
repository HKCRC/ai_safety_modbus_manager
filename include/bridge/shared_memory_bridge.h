#pragma once

#include <cstdint>

#include <boost/signals2/signal.hpp>

#include "ai_safety_common/shared_memory_types.hpp"

struct ModbusConfig;
class TrolleyControl;
class HookWarning;
class MultiTurnEncoderRTU;

struct SharedMemoryBridgeState {
    std::uint8_t last_power_command = 0;
    bool last_alert_3 = false;
    bool last_alert_7 = false;
};

inline boost::signals2::signal<void(ai_safety_common::DeviceStatus)> SignalDeviceStatus;
inline boost::signals2::signal<void(ai_safety_common::CraneState)> SignalCraneState;
inline boost::signals2::signal<void(ai_safety_common::AlertMessage&)> SignalAlert;
inline boost::signals2::signal<void(std::uint8_t&)> SignalPowerButton;

void exchange_shared_memory(const ModbusConfig& config,
                            TrolleyControl& trolley,
                            HookWarning& hook,
                            MultiTurnEncoderRTU* encoder,
                            SharedMemoryBridgeState& state);
