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

double deg_to_rad(double deg) {
    constexpr double kPi = 3.14159265358979323846;
    return deg * kPi / 180.0;
}

float cos_deg(float deg) {
    return static_cast<float>(std::cos(deg_to_rad(static_cast<double>(deg))));
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

    // ============================================================
    // 1. FaultInfo — 异常信息
    ai_safety_common::FaultInfo fault_info{};
    fault_info.timestamp = timestamp_seconds;
    if (!trolley_ok) {
        if (!ping_ipv4_once(config.cab_bridge_ip)) {
            fault_info.category = ai_safety_common::FaultInfo::Category::CabBridgeFault;
        } else if (!ping_ipv4_once(config.trolley_bridge_ip)) {
            fault_info.category = ai_safety_common::FaultInfo::Category::TrolleyBridgeFault;
        } else {
            fault_info.category = ai_safety_common::FaultInfo::Category::TrolleyStm32Fault;
        }
    } else {
        // 小车在线，检查具体硬件通讯状态
        if (!trolley_status.bms_read_ok) {
            fault_info.category = ai_safety_common::FaultInfo::Category::BmsCommunicationFault;
        } else if (!trolley_status.mppt_read_ok) {
            fault_info.category = ai_safety_common::FaultInfo::Category::MpptCommunicationFault;
        } else if (!trolley_status.laser_1_read_ok || !trolley_status.laser_2_read_ok) {
            fault_info.category = ai_safety_common::FaultInfo::Category::LaserCommunicationFault;
        } else if (!trolley_status.cctv_ping_ok) {
            fault_info.category = ai_safety_common::FaultInfo::Category::CctvCommunicationFault;
        }
    }
    SignalFaultInfo(fault_info);

    // ============================================================
    // 2. DeviceStatus — 设备总体状态
    // ============================================================
    const bool is_flat_top =
        (config.crane_type == "flat" || config.crane_type == "flat_top" || config.crane_type == "pingtou");

    ai_safety_common::DeviceStatus device_status{};
    device_status.timestamp = timestamp_seconds;
    device_status.trolleyState = trolley_state_from(trolley_status, trolley_ok);

    if (is_flat_top) {
        device_status.solarCharge = solar_charge_from_mppt(
            trolley_ok ? trolley_status.mppt_charge_status : 0u,
            trolley_ok && (trolley_status.mppt_charge_status != 3u));

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

    // ============================================================
    // 3. CraneState — 距离信息
    // ============================================================
    ai_safety_common::CraneState crane_state{};
    crane_state.timestamp = timestamp_seconds;

    // a. 通过读到的编码器读数，用线性关系计算当前吊钩距离 hookToTrolleyDistanceM
    if (encoder != nullptr) {
        int32_t encoder_position = 0;
        if (encoder->readEncoderPosition(encoder_position)) {
            // 线性关系: y = k * x + b
            crane_state.hookToTrolleyDistanceM =
                config.encoder_k * static_cast<float>(encoder_position) + config.encoder_b;
        }
    }

    // b. 通过读到的两个激光测距距离，计算当前地面距离 groundToTrolleyDistanceM
    if (trolley_ok) {
        const std::uint16_t laser_distance_1 = trolley_status.laser_distance[0];
        const std::uint16_t laser_distance_2 = trolley_status.laser_distance[1];

        const float laser_distance_1_m = static_cast<float>(laser_distance_1) * config.laser_scale_m;
        const float laser_distance_2_m = static_cast<float>(laser_distance_2) * config.laser_scale_m;

        const float corrected_height_1_m =
            (laser_distance_1 != 0u) ? laser_distance_1_m * cos_deg(config.laser1_tilt_deg) : 0.0f;
        const float corrected_height_2_m =
            (laser_distance_2 != 0u) ? laser_distance_2_m * cos_deg(config.laser2_tilt_deg) : 0.0f;

        crane_state.groundToTrolleyDistanceM = std::max(corrected_height_1_m, corrected_height_2_m);
    }

    // 将以上两个数据整合到struct，再通过signal写入共享内存
    SignalCraneState(crane_state);

    // ============================================================
    // 4. AlertMessage — 报警控制
    // ============================================================
    ai_safety_common::AlertMessage alert_message; // 变量声明
    SignalAlert(alert_message); // 这一步之后，alert_message会被填充成最新信息

    // d. 如果Enable3Alert是false，则跳过3m相关警报
    if (alert_message.Enable3Alert) {
        // a. 如果Enable3Alert和Alert3M都是true，则需要向吊钩stm发送3m声音警报和闪灯警报
        if (alert_message.Alert3M != state.last_alert_3) {
            hook.slot_warning(alert_message.Alert3M ? 1 : -1);
            state.last_alert_3 = alert_message.Alert3M;
        }
    }

    // e. 如果Enable7Alert是false，则跳过7m相关警报
    if (alert_message.Enable7Alert) {
        // b. 如果Enable7Alert和Alert7M都是true，则需要向吊钩stm发送7m声音警报和闪灯警报
        // c. 3m和7m有可能同时存在，可以同时发 (通过分别调用slot_warning实现)
        if (alert_message.Alert7M != state.last_alert_7) {
            hook.slot_warning(alert_message.Alert7M ? 2 : -2);
            state.last_alert_7 = alert_message.Alert7M;
        }
    }

    // ============================================================
    // 5. PowerButton — 电源控制
    // ============================================================
    std::uint8_t power_command = 0; // 变量声明
    SignalPowerButton(power_command); // 这一步之后，power_command会被填充成最新信息

    if (trolley_ok) {
        // a. 对比这次读到的和上次读到的数据，检查是否发生变化
        if (power_command != state.last_power_command) {
            // b. 如果发生变化且最新数据是1，则翻译成打开设备电源（CCTV、激光测距）的modbus tcp指令发给小车stm32
            if (power_command == 1u) {
                trolley.setPower3v3(1u);   // 激光测距电源
                trolley.setPowerCctv(1u);  // CCTV电源
            } 
            // c. 如果发生变化且最新数据是2，则翻译成关闭设备电源（CCTV、激光测距）的modbus tcp指令发给小车stm32
            else if (power_command == 2u) {
                trolley.setPowerCctv(0u);
                trolley.setPower3v3(0u);
            }
            state.last_power_command = power_command;
        }
    }
}
