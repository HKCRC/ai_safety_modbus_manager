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

// ── Boilerplate reduction macros (this file only, undefined at EOF) ──

#define MM_BOOL(sig, dev, label, call, err)                                    \
ModbusManagerClient::Status ModbusManagerClient::sig {                         \
    if (!impl_ || !impl_->dev) return {false, label " not initialized"};      \
    bool ok = (call);                                                          \
    return {ok, ok ? "" : label " " err " failed"};                           \
}

#define MM_VOID(sig, dev, label, call)                                         \
ModbusManagerClient::Status ModbusManagerClient::sig {                         \
    if (!impl_ || !impl_->dev) return {false, label " not initialized"};      \
    call;                                                                      \
    return {};                                                                 \
}

// ────────────────────────────────────────────────────────────────────────────

struct ModbusManagerClient::Impl {
    using Status = ModbusManagerClient::Status;

    ModbusConfig config{};
    bool config_loaded = false;

    // ── 核心设备 ──
    std::unique_ptr<TrolleyControl> trolley;
    std::unique_ptr<HookWarning> hook;
    std::unique_ptr<MultiTurnEncoderRTU> encoder;

    // ── 设备轮询线程 ──
    std::thread poll_thread_;
    std::atomic<bool> polling_active_{false};

    void poll_devices_once() {
        if (!trolley) return;
        bridge_.exchange_shared_memory(config, *trolley, hook.get(), encoder.get());
    }

    // ── 共享内存发布 ──
    SharedMemoryBridge bridge_{};

    // ── Hook MQTT（可选，独立于轮询生命周期）──
    std::unique_ptr<HookWarningServer> hook_mqtt;
};

// ============================================================================
//  Lifecycle
// ============================================================================

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

    Status s = initTrolley();
    if (!s.ok) return s;

    {
        if (!impl_->config_loaded) return {false, "config not loaded"};
        impl_->hook = std::make_unique<HookWarning>(impl_->config.hook_dev.c_str(),
                                                    impl_->config.hook_baud,
                                                    impl_->config.hook_parity,
                                                    impl_->config.hook_data_bit,
                                                    impl_->config.hook_stop_bit,
                                                    impl_->config.hook_slave);
        if (!impl_->hook->connect()) {
            std::cerr << "[WARNING] hook connect failed, continuing without hook." << std::endl;
            impl_->hook.reset();
        }
    }

    initEncoder();

    if (!impl_->config.hook_mqtt_device_id.empty()) {
        s = initHookMqtt(impl_->config.hook_mqtt_device_id);
        if (!s.ok) return s;
    }

    return {};
}

// ============================================================================
//  Device init / shutdown / has
// ============================================================================

ModbusManagerClient::Status ModbusManagerClient::initTrolley() {
    if (!impl_) impl_ = std::make_unique<Impl>();
    if (!impl_->config_loaded) return {false, "config not loaded"};

    impl_->trolley = std::make_unique<TrolleyControl>(impl_->config.trolley_ip,
                                                      impl_->config.trolley_port,
                                                      impl_->config.trolley_slave);
    if (!impl_->trolley->connect()) {
        impl_->trolley.reset();
        return {false, "trolley connect failed"};
    }
    return {};
}

ModbusManagerClient::Status ModbusManagerClient::initEncoder() {
    if (!impl_) impl_ = std::make_unique<Impl>();
    if (!impl_->config_loaded) return {false, "config not loaded"};

    if (!impl_->config.encoder_ip.empty()) {
        impl_->encoder = std::make_unique<MultiTurnEncoderRTU>(impl_->config.encoder_ip.c_str(),
                                                               impl_->config.encoder_port,
                                                               impl_->config.encoder_slave);
    } else {
        impl_->encoder = std::make_unique<MultiTurnEncoderRTU>(impl_->config.encoder_dev.c_str(),
                                                               impl_->config.encoder_baud,
                                                               impl_->config.encoder_parity,
                                                               impl_->config.encoder_data_bit,
                                                               impl_->config.encoder_stop_bit,
                                                               impl_->config.encoder_slave);
    }
    if (!impl_->encoder->connect()) {
        std::cerr << "[WARNING] encoder connect failed, continuing without encoder." << std::endl;
        impl_->encoder.reset();
        return {false, "encoder connect failed"};
    }
    return {};
}

void ModbusManagerClient::shutdownTrolley() {
    if (impl_) impl_->trolley.reset();
}

void ModbusManagerClient::shutdownEncoder() {
    if (impl_) impl_->encoder.reset();
}

bool ModbusManagerClient::hasTrolley() const {
    return impl_ && impl_->trolley != nullptr;
}

bool ModbusManagerClient::hasEncoder() const {
    return impl_ && impl_->encoder != nullptr;
}

// ============================================================================
//  Start / Stop
// ============================================================================

ModbusManagerClient::Status ModbusManagerClient::start() {
    if (!impl_) {
        return {false, "not initialized"};
    }
    if (impl_->polling_active_.exchange(true)) {
        return {false, "already running"};
    }

    impl_->poll_thread_ = std::thread([this]() {
        while (impl_->polling_active_.load()) {
            impl_->poll_devices_once();
            std::this_thread::sleep_for(std::chrono::milliseconds(impl_->config.loop_ms));
        }
    });

    return {};
}

ModbusManagerClient::Status ModbusManagerClient::stop() {
    if (!impl_) {
        return {};
    }

    impl_->polling_active_.store(false);

    impl_->bridge_.getSignalDeviceStatus().disconnect_all_slots();
    impl_->bridge_.getSignalFaultInfo().disconnect_all_slots();
    impl_->bridge_.getSignalCraneState().disconnect_all_slots();
    impl_->bridge_.getSignalAlert().disconnect_all_slots();
    impl_->bridge_.getSignalPowerButton().disconnect_all_slots();

    if (impl_->poll_thread_.joinable()) {
        impl_->poll_thread_.join();
    }

    impl_->encoder.reset();
    impl_->hook.reset();
    impl_->trolley.reset();

    impl_->hook_mqtt.reset();

    return {};
}

// ============================================================================
//  Hook MQTT
// ============================================================================

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

// ============================================================================
//  Trolley Control
// ============================================================================

MM_BOOL(triggerTrolleyReboot(),
        trolley, "trolley",
        impl_->trolley->triggerReboot(),
        "triggerReboot")

MM_BOOL(setTrolleyPower3v3(std::uint8_t value),
        trolley, "trolley",
        impl_->trolley->setPower3v3(value),
        "setPower3v3")

MM_BOOL(setTrolleyPower5v(std::uint8_t value),
        trolley, "trolley",
        impl_->trolley->setPower5v(value),
        "setPower5v")

MM_BOOL(setTrolleyPowerCctv(std::uint8_t value),
        trolley, "trolley",
        impl_->trolley->setPowerCctv(value),
        "setPowerCctv")

MM_BOOL(setTrolleyPower4g(std::uint8_t value),
        trolley, "trolley",
        impl_->trolley->setPower4g(value),
        "setPower4g")

MM_BOOL(setTrolleyStandbyEnable(std::uint8_t value),
        trolley, "trolley",
        impl_->trolley->setStandbyEnable(value),
        "setStandbyEnable")

MM_BOOL(setTrolleyStandbyPowerMode(std::uint8_t value),
        trolley, "trolley",
        impl_->trolley->setStandbyPowerMode(value),
        "setStandbyPowerMode")

MM_BOOL(setTrolleySleepMode(std::uint8_t value),
        trolley, "trolley",
        impl_->trolley->setSleepMode(value),
        "setSleepMode")

MM_BOOL(setTrolleyWorkMode(uint16_t mode),
        trolley, "trolley",
        impl_->trolley->setWorkMode(mode),
        "setWorkMode")

MM_BOOL(setTrolleyRtcTime(uint16_t hour, uint16_t minute),
        trolley, "trolley",
        impl_->trolley->setRtcTime(hour, minute),
        "setRtcTime")

MM_BOOL(setTrolleyStartupTime(uint16_t value),
        trolley, "trolley",
        impl_->trolley->setStartupTime(value),
        "setStartupTime")

MM_BOOL(setTrolleyShutdownTime(uint16_t value),
        trolley, "trolley",
        impl_->trolley->setShutdownTime(value),
        "setShutdownTime")

MM_BOOL(getTrolleyStatus(TrolleyStatus& out),
        trolley, "trolley",
        impl_->trolley->readStatus(out),
        "getTrolleyStatus")

// ============================================================================
//  Encoder Control
// ============================================================================

MM_BOOL(readEncoderPosition(int32_t& position),
        encoder, "encoder",
        impl_->encoder->readEncoderPosition(position),
        "readEncoderPosition")

MM_BOOL(readEncoderNumberOfTurns(double& totalTurns, double& time_buffer, double& duration_buffer),
        encoder, "encoder",
        impl_->encoder->readEncoderNumberOfTurns(totalTurns, time_buffer, duration_buffer),
        "readEncoderNumberOfTurns")

MM_BOOL(writeEncoderPosition(int32_t position),
        encoder, "encoder",
        impl_->encoder->writeEncoderPosition(position),
        "writeEncoderPosition")

MM_BOOL(readEncoderSettings(MultiTurnEncoderRTU::EncoderSettings& settings),
        encoder, "encoder",
        impl_->encoder->readEncoderSettings(settings),
        "readEncoderSettings")

MM_BOOL(writeEncoder485DeviceAddress(uint16_t address),
        encoder, "encoder",
        impl_->encoder->write485DeviceAddress(address),
        "writeEncoder485DeviceAddress")

MM_BOOL(writeEncoderBaudRate(MultiTurnEncoderRTU::BaudRate baudRate),
        encoder, "encoder",
        impl_->encoder->writeBaudRate(baudRate),
        "writeEncoderBaudRate")

MM_BOOL(writeEncoderCountingDirection(MultiTurnEncoderRTU::CountingDirection direction),
        encoder, "encoder",
        impl_->encoder->writeCountingDirection(direction),
        "writeEncoderCountingDirection")

MM_BOOL(writeEncoderParityCheck(MultiTurnEncoderRTU::ParityCheck parityCheck),
        encoder, "encoder",
        impl_->encoder->writeParityCheck(parityCheck),
        "writeEncoderParityCheck")

MM_VOID(getEncoderSettingsString(MultiTurnEncoderRTU::EncoderSettingsString& settings_str),
        encoder, "encoder",
        settings_str = impl_->encoder->getEncoderSettings())

MM_VOID(getEncoderData(MultiTurnEncoderRTU::StampedEncoderData& data),
        encoder, "encoder",
        data = impl_->encoder->getEncoderData())

// ============================================================================
//  Shared memory bridge hooks
// ============================================================================

boost::signals2::connection ModbusManagerClient::connectDeviceStatus(
    const std::function<void(ai_safety_common::DeviceStatus)>& slot) {
    if (impl_) {
        return impl_->bridge_.getSignalDeviceStatus().connect(slot);
    }
    return {};
}

boost::signals2::connection ModbusManagerClient::connectFaultInfo(
    const std::function<void(ai_safety_common::FaultInfo)>& slot) {
    if (impl_) {
        return impl_->bridge_.getSignalFaultInfo().connect(slot);
    }
    return {};
}

boost::signals2::connection ModbusManagerClient::connectCraneState(
    const std::function<void(ai_safety_common::CraneState)>& slot) {
    if (impl_) {
        return impl_->bridge_.getSignalCraneState().connect(slot);
    }
    return {};
}

boost::signals2::connection ModbusManagerClient::connectAlert(
    const std::function<void(ai_safety_common::AlertMessage&)>& slot) {
    if (impl_) {
        return impl_->bridge_.getSignalAlert().connect(slot);
    }
    return {};
}

boost::signals2::connection ModbusManagerClient::connectPowerButton(
    const std::function<void(std::uint8_t&)>& slot) {
    if (impl_) {
        return impl_->bridge_.getSignalPowerButton().connect(slot);
    }
    return {};
}

#undef MM_BOOL
#undef MM_VOID
