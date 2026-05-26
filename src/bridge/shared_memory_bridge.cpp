#include "bridge/shared_memory_bridge.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <string>

#include "devices/hookWarning.h"
#include "config/modbus_config.h"
#include "devices/multi_turn_encoder_rtu.h"
#include "devices/trolleyControl.h"

namespace {

double now_seconds() {
    const auto now = std::chrono::system_clock::now().time_since_epoch();
    return std::chrono::duration_cast<std::chrono::duration<double>>(now).count();
}

float cos_deg(float deg) {
    constexpr double kPi = 3.14159265358979323846;
    return static_cast<float>(std::cos(static_cast<double>(deg) * kPi / 180.0));
}

bool is_ipv4_literal(const std::string& ip) {
    if (ip.empty()) {
        return false;
    }
    for (const unsigned char ch : ip) {
        if (!((ch >= '0' && ch <= '9') || ch == '.')) {
            return false;
        }
    }
    return true;
}

bool ping_ipv4_once(const std::string& ip) {
    if (!is_ipv4_literal(ip)) {
        return false;
    }
    const std::string cmd = "ping -c 1 -W 1 " + ip + " > /dev/null 2>&1";
    return std::system(cmd.c_str()) == 0;
}

ai_safety_common::DeviceStatus::SolarChargeState solar_charge_from_mppt(std::uint16_t mppt_status, bool mppt_ok) {
    if (!mppt_ok) {
        return ai_safety_common::DeviceStatus::SolarChargeState::Fault;
    }
    switch (mppt_status) {
        case 0:
            return ai_safety_common::DeviceStatus::SolarChargeState::NotCharging;
        case 1:
            return ai_safety_common::DeviceStatus::SolarChargeState::Charging;
        case 2:
            return ai_safety_common::DeviceStatus::SolarChargeState::Fault;
        default:
            return ai_safety_common::DeviceStatus::SolarChargeState::Unknown;
    }
}

ai_safety_common::DeviceStatus::EquipmentState trolley_state_from(const TrolleyStatus& status, bool trolley_ok) {
    if (!trolley_ok) {
        return ai_safety_common::DeviceStatus::EquipmentState::Offline;
    }
    if (status.device_status == 0u) {
        return ai_safety_common::DeviceStatus::EquipmentState::Standby;
    }
    if (status.device_status == 1u) {
        return ai_safety_common::DeviceStatus::EquipmentState::Active;
    }
    if (status.device_status == 2u) {
        return ai_safety_common::DeviceStatus::EquipmentState::Error;
    }
    return ai_safety_common::DeviceStatus::EquipmentState::Unknown;
}

ai_safety_common::DeviceStatus::EquipmentState hook_state_from(bool hook_ok,
                                                               std::uint16_t workmode,
                                                               std::uint16_t error_code) {
    if (!hook_ok) {
        return ai_safety_common::DeviceStatus::EquipmentState::Offline;
    }
    if (error_code != 0u) {
        return ai_safety_common::DeviceStatus::EquipmentState::Error;
    }
    if (workmode == 0u) {
        return ai_safety_common::DeviceStatus::EquipmentState::Standby;
    }
    return ai_safety_common::DeviceStatus::EquipmentState::Active;
}

} // namespace

void exchange_shared_memory(const ModbusConfig& config,
                            TrolleyControl& trolley,
                            HookWarning& hook,
                            MultiTurnEncoderRTU* encoder,
                            SharedMemoryBridgeState& state) {
    const double timestamp_seconds = now_seconds();

    TrolleyStatus trolley_status{};
    const bool trolley_ok = trolley.readStatus(trolley_status);
    const bool hook_ok = hook.refreshStatus();

    ai_safety_common::DeviceStatus device_status{};
    device_status.timestamp = timestamp_seconds;
    device_status.trolleyState = trolley_state_from(trolley_status, trolley_ok);

    if (!trolley_ok) {
        if (!ping_ipv4_once(config.cab_bridge_ip)) {
            device_status.trolleyOfflineCategory =
                ai_safety_common::DeviceStatus::TrolleyOfflineCategory::CabBridgeFault;
        } else if (!ping_ipv4_once(config.trolley_bridge_ip)) {
            device_status.trolleyOfflineCategory =
                ai_safety_common::DeviceStatus::TrolleyOfflineCategory::TrolleyBridgeFault;
        } else {
            device_status.trolleyOfflineCategory =
                ai_safety_common::DeviceStatus::TrolleyOfflineCategory::TrolleyStm32Fault;
        }
    } else {
        device_status.trolleyOfflineCategory = ai_safety_common::DeviceStatus::TrolleyOfflineCategory::None;
    }

    const bool is_flat_top =
        (config.crane_type == "flat" || config.crane_type == "flat_top" || config.crane_type == "pingtou");

    if (is_flat_top) {
        device_status.solarCharge = solar_charge_from_mppt(
            trolley_ok ? trolley_status.mppt_charge_status : 0u,
            trolley_ok && trolley_status.mppt_read_ok);

        device_status.trolleyBattery.percent = trolley_ok ? trolley_status.battery_level : 0u;
        device_status.trolleyBattery.voltageV = trolley_ok ? trolley_status.battery_voltage_v : 0.0f;
        device_status.trolleyBattery.currentA = trolley_ok ? trolley_status.battery_current_a : 0.0f;

        const bool trolley_is_charging = trolley_ok ? static_cast<bool>(trolley_status.bms_charging) : false;
        device_status.trolleyBattery.isCharging = trolley_is_charging;
        device_status.trolleyBattery.chargingTimeMin =
            (trolley_is_charging && trolley_status.charge_time_valid)
                ? static_cast<std::uint32_t>(trolley_status.charge_time_min)
                : 0u;
        device_status.trolleyBattery.remainingMin =
            (!trolley_is_charging && trolley_status.discharge_time_valid)
                ? static_cast<std::uint32_t>(trolley_status.discharge_time_min)
                : 0u;
    }

    const std::uint16_t hook_workmode = hook.getWorkmode();
    const std::uint16_t hook_error = hook.getErrorCode();
    device_status.hookState = hook_state_from(hook_ok, hook_workmode, hook_error);
    device_status.hookBattery.percent =
        hook_ok ? static_cast<std::uint8_t>(hook.get_battery_level_feedback()) : 0u;
    device_status.hookBattery.remainingMin =
        hook_ok ? static_cast<std::uint32_t>(hook.getDischargeTime()) : 0u;
    device_status.hookBattery.isCharging = hook_ok ? (hook.getChargingStatus() != 0u) : false;
    device_status.hookBattery.chargingTimeMin =
        hook_ok ? static_cast<std::uint32_t>(hook.getChargeTime()) : 0u;
    device_status.hookBattery.voltageV = hook_ok ? hook.getVoltage() : 0.0f;
    device_status.hookBattery.currentA = hook_ok ? hook.getCurrent() : 0.0f;

    SignalDeviceStatus(device_status);

    ai_safety_common::CraneState crane_state{};
    crane_state.timestamp = timestamp_seconds;
    crane_state.hookToTrolleyDistanceM = 0.0f;

    if (encoder != nullptr) {
        int32_t encoder_position = 0;
        if (encoder->readEncoderPosition(encoder_position)) {
            crane_state.hookToTrolleyDistanceM =
                config.encoder_k * static_cast<float>(encoder_position) + config.encoder_b;
        }
    }

    float ground_m = 0.0f;
    if (trolley_ok) {
        const std::uint16_t laser_distance_1 = trolley_status.laser_distance[0];
        const std::uint16_t laser_distance_2 = trolley_status.laser_distance[1];

        const float laser_distance_1_m = static_cast<float>(laser_distance_1) * config.laser_scale_m;
        const float laser_distance_2_m = static_cast<float>(laser_distance_2) * config.laser_scale_m;

        const float corrected_height_1_m =
            (laser_distance_1 != 0u) ? laser_distance_1_m * cos_deg(config.laser1_tilt_deg) : 0.0f;
        const float corrected_height_2_m =
            (laser_distance_2 != 0u) ? laser_distance_2_m * cos_deg(config.laser2_tilt_deg) : 0.0f;

        ground_m = std::max(corrected_height_1_m, corrected_height_2_m);
    }
    crane_state.groundToTrolleyDistanceM = ground_m;

    SignalCraneState(crane_state);

    ai_safety_common::AlertMessage alert{};
    SignalAlert(alert);

    if (alert.Enable3Alert) {
        const bool desired_alert_3m = alert.Alert3M;
        if (desired_alert_3m != state.last_alert_3) {
            hook.slot_warning(desired_alert_3m ? 1 : -1);
            state.last_alert_3 = desired_alert_3m;
        }
    }
    if (alert.Enable7Alert) {
        const bool desired_alert_7m = alert.Alert7M;
        if (desired_alert_7m != state.last_alert_7) {
            hook.slot_warning(desired_alert_7m ? 2 : -2);
            state.last_alert_7 = desired_alert_7m;
        }
    }

    if (trolley_ok) {
        std::uint8_t power_command = 0;
        SignalPowerButton(power_command);
        if (power_command != state.last_power_command) {
            if (power_command == 1u) {
                trolley.setPower3v3(1u);
                trolley.setPowerCctv(1u);
            } else if (power_command == 2u) {
                trolley.setPowerCctv(0u);
                trolley.setPower3v3(0u);
            }
            state.last_power_command = power_command;
        }
    }
}
