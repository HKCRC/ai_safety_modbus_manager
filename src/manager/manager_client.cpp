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
    impl_->running.store(false);
    if (impl_->worker.joinable()) {
        impl_->worker.join();
    }
    impl_->encoder.reset();
    impl_->hook.reset();
    impl_->trolley.reset();
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
