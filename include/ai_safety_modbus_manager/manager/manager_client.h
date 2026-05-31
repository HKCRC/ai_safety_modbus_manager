#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <boost/signals2/connection.hpp>
#include "ai_safety_common/shared_memory_types.hpp"
#include "devices/multi_turn_encoder_rtu.h"
#include "devices/trolleyControl.h"
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

    // Trolley Control
    Status triggerTrolleyReboot();
    Status setTrolleyPower3v3(std::uint8_t value);
    Status setTrolleyPower5v(std::uint8_t value);
    Status setTrolleyPowerCctv(std::uint8_t value);
    Status setTrolleyPower4g(std::uint8_t value);
    Status setTrolleyStandbyEnable(std::uint8_t value);
    Status setTrolleyStandbyPowerMode(std::uint8_t value);
    Status setTrolleySleepMode(std::uint8_t value);
    Status setTrolleyWorkMode(uint16_t mode);
    Status setTrolleyRtcTime(uint16_t hour, uint16_t minute);
    Status setTrolleyStartupTime(uint16_t value);
    Status setTrolleyShutdownTime(uint16_t value);
    Status getTrolleyStatus(TrolleyStatus& out);

    // Encoder Control
    Status readEncoderPosition(int32_t& position);
    Status readEncoderNumberOfTurns(double& totalTurns, double& time_buffer, double& duration_buffer);
    Status writeEncoderPosition(int32_t position);
    Status readEncoderSettings(MultiTurnEncoderRTU::EncoderSettings& settings);
    Status writeEncoder485DeviceAddress(uint16_t address);
    Status writeEncoderBaudRate(MultiTurnEncoderRTU::BaudRate baudRate);
    Status writeEncoderCountingDirection(MultiTurnEncoderRTU::CountingDirection direction);
    Status writeEncoderParityCheck(MultiTurnEncoderRTU::ParityCheck parityCheck);
    Status getEncoderSettingsString(MultiTurnEncoderRTU::EncoderSettingsString& settings_str);
    Status getEncoderData(MultiTurnEncoderRTU::StampedEncoderData& data);

    // Shared memory bridge hooks
    boost::signals2::connection connectDeviceStatus(
        const std::function<void(ai_safety_common::DeviceStatus)>& slot);

    boost::signals2::connection connectFaultInfo(
        const std::function<void(ai_safety_common::FaultInfo)>& slot);

    boost::signals2::connection connectCraneState(
        const std::function<void(ai_safety_common::CraneState)>& slot);

    boost::signals2::connection connectAlert(
        const std::function<void(ai_safety_common::AlertMessage&)>& slot);

    boost::signals2::connection connectPowerButton(
        const std::function<void(std::uint8_t&)>& slot);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
