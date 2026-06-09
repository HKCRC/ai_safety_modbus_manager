#pragma once

#include <cstdint>

#include <boost/signals2/signal.hpp>

#include "ai_safety_common/shared_memory_types.hpp"

struct ModbusConfig;
struct TrolleyStatus;
class TrolleyControl;
class HookWarningServer;
class MultiTurnEncoderRTU;

class SharedMemoryBridge {
public:
    using SignalDeviceStatusType = boost::signals2::signal<void(ai_safety_common::DeviceStatus)>;
    using SignalTrolleyConnectFaultInfoType =
        boost::signals2::signal<void(ai_safety_common::TrolleyConnectFaultInfo)>;
    using SignalTrolleyDevicesErrorInfoType =
        boost::signals2::signal<void(ai_safety_common::TrolleyDevicesErrorInfo)>;
    using SignalCraneStateType = boost::signals2::signal<void(ai_safety_common::CraneState)>;
    using SignalAlertType = boost::signals2::signal<void(ai_safety_common::AlertMessage&)>;
    using SignalPowerButtonType = boost::signals2::signal<void(std::uint8_t&)>;

    SharedMemoryBridge() = default;
    ~SharedMemoryBridge() = default;

    void exchange_shared_memory(const ModbusConfig& config,
                                TrolleyControl* trolley,
                                HookWarningServer* hook,
                                MultiTurnEncoderRTU* encoder);

    SignalDeviceStatusType& getSignalDeviceStatus() { return signal_device_status_; }
    SignalTrolleyConnectFaultInfoType& getSignalTrolleyConnectFaultInfo() {
        return signal_trolley_connect_fault_info_;
    }
    SignalTrolleyDevicesErrorInfoType& getSignalTrolleyDevicesErrorInfo() {
        return signal_trolley_devices_error_info_;
    }
    SignalCraneStateType& getSignalCraneState() { return signal_crane_state_; }
    SignalAlertType& getSignalAlert() { return signal_alert_; }
    SignalPowerButtonType& getSignalPowerButton() { return signal_power_button_; }

private:
    static constexpr std::uint8_t kDefaultAlertVolume = 30u;

    static double now_seconds();
    static double deg_to_rad(double deg);
    static float cos_deg(float deg);
    static bool is_ipv4_literal(const std::string& ip);
    static bool ping_ipv4_once(const std::string& ip);

    static ai_safety_common::DeviceStatus::SolarChargeState solar_charge_from_mppt(
        std::uint16_t mppt_status, bool mppt_ok);

    static ai_safety_common::DeviceStatus::EquipmentState trolley_state_from(
        const TrolleyStatus& status, bool trolley_ok);

    static ai_safety_common::DeviceStatus::EquipmentState hook_state_from(
        bool hook_ok, std::uint16_t workmode);

    SignalDeviceStatusType signal_device_status_;
    SignalTrolleyConnectFaultInfoType signal_trolley_connect_fault_info_;
    SignalTrolleyDevicesErrorInfoType signal_trolley_devices_error_info_;
    SignalCraneStateType signal_crane_state_;
    SignalAlertType signal_alert_;
    SignalPowerButtonType signal_power_button_;

    std::uint8_t last_power_command_ = 0;
    bool last_alert_3_ = false;
    bool last_alert_7_ = false;
};
