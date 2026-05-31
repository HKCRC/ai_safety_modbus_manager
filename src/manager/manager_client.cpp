#include "manager/manager_client.h"

#include <atomic>
#include <chrono>
#include <iostream>
#include <exception>
#include <thread>
#include <utility>

#include "devices/hookWarning/hookWarning.h"
#include "devices/hookWarning/hookWarning_mqtt.h"
#include "config/modbus_config.h"
#include "devices/multi_turn_encoder_rtu.h"
#include "bridge/shared_memory_bridge.h"
#include "devices/trolleyControl.h"

struct ModbusManagerClient::Impl {
    using Status = ModbusManagerClient::Status;

    ModbusConfig config{};
    bool config_loaded = false;

    std::unique_ptr<TrolleyControl> trolley;
    std::unique_ptr<HookWarning> hook;
    std::unique_ptr<MultiTurnEncoderRTU> encoder;
    std::unique_ptr<HookWarningServer> hook_mqtt;

    std::thread worker;
    std::atomic<bool> running{false};
    SharedMemoryBridge bridge{};

    void publish_once() {
        bridge.exchange_shared_memory(config, *trolley, *hook, encoder.get());
    }
};

ModbusManagerClient::ModbusManagerClient() : impl_(std::make_unique<Impl>()) {}

ModbusManagerClient::~ModbusManagerClient() {
    stop();
}

ModbusManagerClient::Status ModbusManagerClient::loadConfig(const std::string& path) {
    if (!impl_) {
        impl_ = std::make_unique<Impl>();
    }
    if (!impl_->config.loadFromFile(path)) {
        return {false, "Failed to load config from " + path};
    }
    impl_->config_loaded = true;
    return {};
}

ModbusManagerClient::Status ModbusManagerClient::init() {
    if (!impl_) {
        impl_ = std::make_unique<Impl>();
    }
    if (!impl_->config_loaded) {
        return {false, "config not loaded"};
    }

    impl_->trolley = std::make_unique<TrolleyControl>(impl_->config.trolley_ip,
                                                      impl_->config.trolley_port,
                                                      impl_->config.trolley_slave);
    if (!impl_->trolley->connect()) {
        return {false, "trolley connect failed"};
    }

    impl_->hook = std::make_unique<HookWarning>(impl_->config.hook_dev.c_str(),
                                                impl_->config.hook_baud,
                                                impl_->config.hook_parity,
                                                impl_->config.hook_data_bit,
                                                impl_->config.hook_stop_bit,
                                                impl_->config.hook_slave);
    if (!impl_->hook->connect()) {
        std::cerr << "[WARNING] hook connect failed, continuing without hook." << std::endl;
    }

    impl_->encoder = std::make_unique<MultiTurnEncoderRTU>(impl_->config.encoder_dev.c_str(),
                                                           impl_->config.encoder_baud,
                                                           impl_->config.encoder_parity,
                                                           impl_->config.encoder_data_bit,
                                                           impl_->config.encoder_stop_bit,
                                                           impl_->config.encoder_slave);
    if (!impl_->encoder->connect()) {
        std::cerr << "[WARNING] encoder connect failed, continuing without encoder." << std::endl;
    }

    if (!impl_->config.hook_mqtt_device_id.empty()) {
        const Status hook_mqtt_status = initHookMqtt(impl_->config.hook_mqtt_device_id);
        if (!hook_mqtt_status.ok) {
            return hook_mqtt_status;
        }
    }

    return {};
}

ModbusManagerClient::Status ModbusManagerClient::start() {
    if (!impl_) {
        return {false, "not initialized"};
    }
    if (impl_->running.exchange(true)) {
        return {false, "already running"};
    }

    impl_->worker = std::thread([this]() {
        while (impl_->running.load()) {
            impl_->publish_once();
            std::this_thread::sleep_for(std::chrono::milliseconds(impl_->config.loop_ms));
        }
    });

    return {};
}

ModbusManagerClient::Status ModbusManagerClient::stop() {
    if (!impl_) {
        return {};
    }
    
    // 先标记为不在运行，这样 worker 线程就不会再继续循环
    impl_->running.store(false);
    
    // 断开所有 signal 绑定，防止 worker 线程最后一次执行或有延迟的 signal 触发
    impl_->bridge.getSignalDeviceStatus().disconnect_all_slots();
    impl_->bridge.getSignalFaultInfo().disconnect_all_slots();
    impl_->bridge.getSignalCraneState().disconnect_all_slots();
    impl_->bridge.getSignalAlert().disconnect_all_slots();
    impl_->bridge.getSignalPowerButton().disconnect_all_slots();

    if (impl_->worker.joinable()) {
        impl_->worker.join();
    }
    
    // 按顺序释放资源
    impl_->encoder.reset();
    impl_->hook.reset();
    impl_->trolley.reset();
    
    // 最后释放 mqtt，避免 "disconnect attempt too soon" 等问题
    impl_->hook_mqtt.reset();
    
    return {};
}

ModbusManagerClient::Status ModbusManagerClient::initHookMqtt(const std::string& device_id) {
    if (device_id.empty()) {
        return {false, "hook mqtt device_id is empty"};
    }
    if (!impl_) {
        impl_ = std::make_unique<Impl>();
    }

    try {
        impl_->hook_mqtt = std::make_unique<HookWarningServer>(device_id);
    } catch (const std::exception& ex) {
        impl_->hook_mqtt.reset();
        return {false, std::string("failed to init hook mqtt: ") + ex.what()};
    } catch (...) {
        impl_->hook_mqtt.reset();
        return {false, "failed to init hook mqtt"};
    }
    return {};
}

ModbusManagerClient::Status ModbusManagerClient::resetHookMqtt() {
    if (!impl_) {
        return {};
    }
    impl_->hook_mqtt.reset();
    return {};
}

ModbusManagerClient::Status ModbusManagerClient::setHookMqttFlashLight(
    bool light_on, bool sound_7m, bool sound_3m, std::uint8_t volume) {
    if (!impl_ || !impl_->hook_mqtt) {
        return {false, "hook mqtt not initialized"};
    }
    impl_->hook_mqtt->set_flash_light(light_on, sound_7m, sound_3m, volume);
    return {};
}

ModbusManagerClient::Status ModbusManagerClient::setHookMqttSysMode(
    bool is_standby, std::uint8_t work_mode) {
    if (!impl_ || !impl_->hook_mqtt) {
        return {false, "hook mqtt not initialized"};
    }
    impl_->hook_mqtt->set_sys_mode(is_standby, work_mode);
    return {};
}

ModbusManagerClient::Status ModbusManagerClient::setHookMqttTimeSchedule(
    std::uint8_t off_hour,
    std::uint8_t off_minute,
    std::uint8_t on_hour,
    std::uint8_t on_minute) {
    if (!impl_ || !impl_->hook_mqtt) {
        return {false, "hook mqtt not initialized"};
    }
    if (off_hour > 23 || on_hour > 23 || off_minute > 59 || on_minute > 59) {
        return {false, "invalid hook mqtt schedule time"};
    }
    impl_->hook_mqtt->set_time_schedule(off_hour, off_minute, on_hour, on_minute);
    return {};
}

ModbusManagerClient::Status ModbusManagerClient::getHookMqttBmsStatus(BmsStatusData& status) {
    if (!impl_ || !impl_->hook_mqtt) {
        return {false, "hook mqtt not initialized"};
    }
    status = impl_->hook_mqtt->get_bms_status();
    return {};
}

ModbusManagerClient::Status ModbusManagerClient::getHookMqttLightStatus(
    FlashLightStatusData& status) {
    if (!impl_ || !impl_->hook_mqtt) {
        return {false, "hook mqtt not initialized"};
    }
    status = impl_->hook_mqtt->get_light_status();
    return {};
}

ModbusManagerClient::Status ModbusManagerClient::getHookMqttSysStatus(SysStatusData& status) {
    if (!impl_ || !impl_->hook_mqtt) {
        return {false, "hook mqtt not initialized"};
    }
    status = impl_->hook_mqtt->get_sys_status();
    return {};
}

ModbusManagerClient::Status ModbusManagerClient::getHookMqttScheduleStatus(
    SysScheduleData& status) {
    if (!impl_ || !impl_->hook_mqtt) {
        return {false, "hook mqtt not initialized"};
    }
    status = impl_->hook_mqtt->get_schedule_status();
    return {};
}

// ==========================================
// Trolley Control
// ==========================================
ModbusManagerClient::Status ModbusManagerClient::triggerTrolleyReboot() {
    if (!impl_ || !impl_->trolley) return {false, "trolley not initialized"};
    bool ok = impl_->trolley->triggerReboot();
    return {ok, ok ? "" : "trolley triggerReboot failed"};
}
ModbusManagerClient::Status ModbusManagerClient::setTrolleyPower3v3(std::uint8_t value) {
    if (!impl_ || !impl_->trolley) return {false, "trolley not initialized"};
    bool ok = impl_->trolley->setPower3v3(value);
    return {ok, ok ? "" : "trolley setPower3v3 failed"};
}
ModbusManagerClient::Status ModbusManagerClient::setTrolleyPower5v(std::uint8_t value) {
    if (!impl_ || !impl_->trolley) return {false, "trolley not initialized"};
    bool ok = impl_->trolley->setPower5v(value);
    return {ok, ok ? "" : "trolley setPower5v failed"};
}
ModbusManagerClient::Status ModbusManagerClient::setTrolleyPowerCctv(std::uint8_t value) {
    if (!impl_ || !impl_->trolley) return {false, "trolley not initialized"};
    bool ok = impl_->trolley->setPowerCctv(value);
    return {ok, ok ? "" : "trolley setPowerCctv failed"};
}
ModbusManagerClient::Status ModbusManagerClient::setTrolleyPower4g(std::uint8_t value) {
    if (!impl_ || !impl_->trolley) return {false, "trolley not initialized"};
    bool ok = impl_->trolley->setPower4g(value);
    return {ok, ok ? "" : "trolley setPower4g failed"};
}
ModbusManagerClient::Status ModbusManagerClient::setTrolleyStandbyEnable(std::uint8_t value) {
    if (!impl_ || !impl_->trolley) return {false, "trolley not initialized"};
    bool ok = impl_->trolley->setStandbyEnable(value);
    return {ok, ok ? "" : "trolley setStandbyEnable failed"};
}
ModbusManagerClient::Status ModbusManagerClient::setTrolleyStandbyPowerMode(std::uint8_t value) {
    if (!impl_ || !impl_->trolley) return {false, "trolley not initialized"};
    bool ok = impl_->trolley->setStandbyPowerMode(value);
    return {ok, ok ? "" : "trolley setStandbyPowerMode failed"};
}
ModbusManagerClient::Status ModbusManagerClient::setTrolleySleepMode(std::uint8_t value) {
    if (!impl_ || !impl_->trolley) return {false, "trolley not initialized"};
    bool ok = impl_->trolley->setSleepMode(value);
    return {ok, ok ? "" : "trolley setSleepMode failed"};
}
ModbusManagerClient::Status ModbusManagerClient::setTrolleyWorkMode(uint16_t mode) {
    if (!impl_ || !impl_->trolley) return {false, "trolley not initialized"};
    bool ok = impl_->trolley->setWorkMode(mode);
    return {ok, ok ? "" : "trolley setWorkMode failed"};
}
ModbusManagerClient::Status ModbusManagerClient::setTrolleyRtcTime(uint16_t hour, uint16_t minute) {
    if (!impl_ || !impl_->trolley) return {false, "trolley not initialized"};
    bool ok = impl_->trolley->setRtcTime(hour, minute);
    return {ok, ok ? "" : "trolley setRtcTime failed"};
}
ModbusManagerClient::Status ModbusManagerClient::setTrolleyStartupTime(uint16_t value) {
    if (!impl_ || !impl_->trolley) return {false, "trolley not initialized"};
    bool ok = impl_->trolley->setStartupTime(value);
    return {ok, ok ? "" : "trolley setStartupTime failed"};
}
ModbusManagerClient::Status ModbusManagerClient::setTrolleyShutdownTime(uint16_t value) {
    if (!impl_ || !impl_->trolley) return {false, "trolley not initialized"};
    bool ok = impl_->trolley->setShutdownTime(value);
    return {ok, ok ? "" : "trolley setShutdownTime failed"};
}
ModbusManagerClient::Status ModbusManagerClient::getTrolleyStatus(TrolleyStatus& out) {
    if (!impl_ || !impl_->trolley) return {false, "trolley not initialized"};
    bool ok = impl_->trolley->readStatus(out);
    return {ok, ok ? "" : "trolley readStatus failed"};
}

// ==========================================
// Encoder Control
// ==========================================
ModbusManagerClient::Status ModbusManagerClient::readEncoderPosition(int32_t& position) {
    if (!impl_ || !impl_->encoder) return {false, "encoder not initialized"};
    bool ok = impl_->encoder->readEncoderPosition(position);
    return {ok, ok ? "" : "encoder readEncoderPosition failed"};
}
ModbusManagerClient::Status ModbusManagerClient::readEncoderNumberOfTurns(double& totalTurns, double& time_buffer, double& duration_buffer) {
    if (!impl_ || !impl_->encoder) return {false, "encoder not initialized"};
    bool ok = impl_->encoder->readEncoderNumberOfTurns(totalTurns, time_buffer, duration_buffer);
    return {ok, ok ? "" : "encoder readEncoderNumberOfTurns failed"};
}
ModbusManagerClient::Status ModbusManagerClient::writeEncoderPosition(int32_t position) {
    if (!impl_ || !impl_->encoder) return {false, "encoder not initialized"};
    bool ok = impl_->encoder->writeEncoderPosition(position);
    return {ok, ok ? "" : "encoder writeEncoderPosition failed"};
}
ModbusManagerClient::Status ModbusManagerClient::readEncoderSettings(MultiTurnEncoderRTU::EncoderSettings& settings) {
    if (!impl_ || !impl_->encoder) return {false, "encoder not initialized"};
    bool ok = impl_->encoder->readEncoderSettings(settings);
    return {ok, ok ? "" : "encoder readEncoderSettings failed"};
}
ModbusManagerClient::Status ModbusManagerClient::writeEncoder485DeviceAddress(uint16_t address) {
    if (!impl_ || !impl_->encoder) return {false, "encoder not initialized"};
    bool ok = impl_->encoder->write485DeviceAddress(address);
    return {ok, ok ? "" : "encoder write485DeviceAddress failed"};
}
ModbusManagerClient::Status ModbusManagerClient::writeEncoderBaudRate(MultiTurnEncoderRTU::BaudRate baudRate) {
    if (!impl_ || !impl_->encoder) return {false, "encoder not initialized"};
    bool ok = impl_->encoder->writeBaudRate(baudRate);
    return {ok, ok ? "" : "encoder writeBaudRate failed"};
}
ModbusManagerClient::Status ModbusManagerClient::writeEncoderCountingDirection(MultiTurnEncoderRTU::CountingDirection direction) {
    if (!impl_ || !impl_->encoder) return {false, "encoder not initialized"};
    bool ok = impl_->encoder->writeCountingDirection(direction);
    return {ok, ok ? "" : "encoder writeCountingDirection failed"};
}
ModbusManagerClient::Status ModbusManagerClient::writeEncoderParityCheck(MultiTurnEncoderRTU::ParityCheck parityCheck) {
    if (!impl_ || !impl_->encoder) return {false, "encoder not initialized"};
    bool ok = impl_->encoder->writeParityCheck(parityCheck);
    return {ok, ok ? "" : "encoder writeParityCheck failed"};
}
ModbusManagerClient::Status ModbusManagerClient::getEncoderSettingsString(MultiTurnEncoderRTU::EncoderSettingsString& settings_str) {
    if (!impl_ || !impl_->encoder) return {false, "encoder not initialized"};
    settings_str = impl_->encoder->getEncoderSettings();
    return {};
}
ModbusManagerClient::Status ModbusManagerClient::getEncoderData(MultiTurnEncoderRTU::StampedEncoderData& data) {
    if (!impl_ || !impl_->encoder) return {false, "encoder not initialized"};
    data = impl_->encoder->getEncoderData();
    return {};
}

boost::signals2::connection ModbusManagerClient::connectDeviceStatus(
    const std::function<void(ai_safety_common::DeviceStatus)>& slot) {
    if (impl_) {
        return impl_->bridge.getSignalDeviceStatus().connect(slot);
    }
    return {};
}

boost::signals2::connection ModbusManagerClient::connectFaultInfo(
    const std::function<void(ai_safety_common::FaultInfo)>& slot) {
    if (impl_) {
        return impl_->bridge.getSignalFaultInfo().connect(slot);
    }
    return {};
}

boost::signals2::connection ModbusManagerClient::connectCraneState(
    const std::function<void(ai_safety_common::CraneState)>& slot) {
    if (impl_) {
        return impl_->bridge.getSignalCraneState().connect(slot);
    }
    return {};
}

boost::signals2::connection ModbusManagerClient::connectAlert(
    const std::function<void(ai_safety_common::AlertMessage&)>& slot) {
    if (impl_) {
        return impl_->bridge.getSignalAlert().connect(slot);
    }
    return {};
}

boost::signals2::connection ModbusManagerClient::connectPowerButton(
    const std::function<void(std::uint8_t&)>& slot) {
    if (impl_) {
        return impl_->bridge.getSignalPowerButton().connect(slot);
    }
    return {};
}
