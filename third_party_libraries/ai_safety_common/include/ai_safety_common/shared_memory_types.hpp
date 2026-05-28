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
    BatteryInfo trolleyBattery;

    EquipmentState hookState = EquipmentState::Unknown;
    BatteryInfo hookBattery;

    double timestamp = 0.0;
};

struct FaultInfo {
    enum class Category : std::uint8_t {
        None = 0,
        CabBridgeFault,
        TrolleyBridgeFault,
        TrolleyStm32Fault,
        BmsCommunicationFault,
        MpptCommunicationFault,
        LaserCommunicationFault,
        CctvCommunicationFault
    };

    Category category = Category::None;
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

struct JoystickControlData {
    enum class PowerCommand : std::uint8_t {
        None = 0,
        PowerOn,
        PowerOff
    };

    // Optical zoom with 2 discrete levels.
    enum class OpticalZoomLevel : std::uint8_t {
        Level0 = 0,
        Level1
    };

    // Digital zoom ratio relative to the original image:
    // 1.0 = no digital zoom, > 1.0 = zoom in. Should be >= 1.0.
    float digitalZoomRatio = 1.0f;
    std::uint32_t CenterPixelX = 0;
    std::uint32_t CenterPixelY = 0;

    PowerCommand powerCommand = PowerCommand::None;
    OpticalZoomLevel opticalZoom = OpticalZoomLevel::Level0;

    double timestamp = 0.0;
};

// Customer-facing parameters that can be tuned online (exposed via shared memory).
struct CustomerTuningParams {
    // Local time-of-day (hour: 0-23, minute: 0-59).
    struct TimeOfDay {
        std::uint8_t hour = 0;
        std::uint8_t minute = 0;
    };

    bool enable3mDetection = true;
    bool enable7mDetection = true;
    bool enableHookAlert = true;

    // Hook alert speaker volume, range 0-100 (same scale as devices_manager speaker_volume).
    std::uint8_t hookAlertVolume = 100;

    TimeOfDay trolleySleepStart{19, 0};
    TimeOfDay trolleySleepEnd{7, 0};

    double timestamp = 0.0;
};

}  // namespace ai_safety_common
