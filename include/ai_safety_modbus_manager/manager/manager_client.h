#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <boost/signals2/connection.hpp>
#include "ai_safety_common/shared_memory_types.hpp"
#include "devices/hookWarning/hookWarning_mqtt.h"
class ModbusManagerClient {
public:
    struct Status {
        bool ok = true;
        std::string message;
    };

    // Basic lifecycle
    ModbusManagerClient();
    ~ModbusManagerClient();

    Status loadConfig(const std::string& path);
    Status init();
    Status start();
    Status stop();

    // Hook MQTT
    Status initHookMqtt(const std::string& device_id);
    Status resetHookMqtt();

    Status setHookMqttFlashLight(bool light_on,
                                 bool sound_7m,
                                 bool sound_3m,
                                 std::uint8_t volume);
    Status setHookMqttSysMode(bool is_standby, std::uint8_t work_mode);
    Status setHookMqttTimeSchedule(std::uint8_t off_hour,
                                   std::uint8_t off_minute,
                                   std::uint8_t on_hour,
                                   std::uint8_t on_minute);

    Status getHookMqttBmsStatus(BmsStatusData& status);
    Status getHookMqttLightStatus(FlashLightStatusData& status);
    Status getHookMqttSysStatus(SysStatusData& status);
    Status getHookMqttScheduleStatus(SysScheduleData& status);

    // Shared memory bridge hooks
    // 设备总体状态桥接回调
    boost::signals2::connection connectDeviceStatus(
        const std::function<void(ai_safety_common::DeviceStatus)>& slot);

    // 异常故障信息桥接回调
    boost::signals2::connection connectFaultInfo(
        const std::function<void(ai_safety_common::FaultInfo)>& slot);

    // 距离状态桥接回调
    boost::signals2::connection connectCraneState(
        const std::function<void(ai_safety_common::CraneState)>& slot);

    // 报警控制交换钩子，可修改消息后参与后续写入
    boost::signals2::connection connectAlert(
        const std::function<void(ai_safety_common::AlertMessage&)>& slot);

    // 电源控制交换钩子，可修改命令后参与后续写入
    boost::signals2::connection connectPowerButton(
        const std::function<void(std::uint8_t&)>& slot);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
