#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <boost/signals2/connection.hpp>
#include "ai_safety_common/shared_memory_types.hpp"
class ModbusManagerClient {
public:
    struct Status {
        bool ok = true;
        std::string message;
    };

    ModbusManagerClient();
    Status loadConfig(const std::string& path);
    Status init();
    Status start();
    Status stop();

    ~ModbusManagerClient();
// 1. 监听设备总体状态信号
    boost::signals2::connection connectDeviceStatus(
        const std::function<void(ai_safety_common::DeviceStatus)>& slot);

    // 2. 监听异常故障信息信号
    boost::signals2::connection connectFaultInfo(
        const std::function<void(ai_safety_common::FaultInfo)>& slot);

    // 3. 监听起重机/距离状态信号
    boost::signals2::connection connectCraneState(
        const std::function<void(ai_safety_common::CraneState)>& slot);

    // 4. 报警控制信号（注意是引用传递！外部槽函数可以通过修改 alert 从而控制硬件警告）
    boost::signals2::connection connectAlert(
        const std::function<void(ai_safety_common::AlertMessage&)>& slot);

    // 5. 电源控制信号（注意是引用传递！外部槽函数可以通过修改 power_command 控制 cctv / 3v3 电源）
    boost::signals2::connection connectPowerButton(
        const std::function<void(std::uint8_t&)>& slot);
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
