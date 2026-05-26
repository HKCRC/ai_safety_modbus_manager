#pragma once

#include <cstdint>

namespace ai_safety_common {

struct DeviceStatus {
    enum class SolarChargeState : std::uint8_t {
        Unknown = 0,
        NotCharging,
        Charging,
        Fault
    };

    enum class EquipmentState : std::uint8_t {
        Unknown = 0,
        Offline,
        Standby,
        Active,
        Error
    };

    enum class TrolleyOfflineCategory : std::uint8_t {
        None = 0,
        CabBridgeFault,
        TrolleyBridgeFault,
        TrolleyStm32Fault
    };

    struct BatteryInfo {
        std::uint8_t percent = 0;
        std::uint32_t remainingMin = 0;
        bool isCharging = false;
        std::uint32_t chargingTimeMin = 0;
        float voltageV = 0.0f;
        float currentA = 0.0f;
    };

    SolarChargeState solarCharge = SolarChargeState::Unknown;

    EquipmentState trolleyState = EquipmentState::Unknown;
    TrolleyOfflineCategory trolleyOfflineCategory = TrolleyOfflineCategory::None;
    BatteryInfo trolleyBattery;

    EquipmentState hookState = EquipmentState::Unknown;
    BatteryInfo hookBattery;

    double timestamp = 0.0;
};

struct CraneState {
    double timestamp = 0.0;
    float hookToTrolleyDistanceM = 0.0f;
    float groundToTrolleyDistanceM = 0.0f;
};

struct AlertMessage {
    double timestamp = 0.0;
    bool Enable3Alert = true;
    bool Enable7Alert = true;
    bool Alert3M = false;
    bool Alert7M = false;
};

}  // namespace ai_safety_common

