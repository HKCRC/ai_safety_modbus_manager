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
    static constexpr auto kFailLogInterval = std::chrono::seconds(30);
    static constexpr auto kReconnectInterval = std::chrono::seconds(2);
    std::chrono::steady_clock::time_point last_trolley_fail_log_{};
    std::chrono::steady_clock::time_point last_encoder_fail_log_{};

    // ── 核心设备 ──
    std::unique_ptr<TrolleyControl> trolley;
    std::unique_ptr<MultiTurnEncoderRTU> encoder;

    // ── 设备轮询线程 ──
    std::thread poll_thread_;
    std::atomic<bool> polling_active_{false};
    std::chrono::steady_clock::time_point next_trolley_reconnect_at_{};
    std::chrono::steady_clock::time_point next_encoder_reconnect_at_{};
    std::chrono::steady_clock::time_point next_hook_mqtt_reconnect_at_{};

    bool try_init_trolley(bool log_failure = true) {
        trolley = std::make_unique<TrolleyControl>(config.trolley_ip,
                                                   config.trolley_port,
                                                   config.trolley_slave);
        if (!log_failure) trolley->setDisableCerr(true);
        if (!trolley->connect()) {
            trolley.reset();
            if (log_failure) {
                std::cerr << "[WARNING] trolley connect failed, continuing without trolley." << std::endl;
            }
            return false;
        }
        std::cout << "[INFO] trolley connected." << std::endl;
        return true;
    }



    bool try_init_encoder(bool log_failure = true) {
        if (!config.encoder_ip.empty()) {
            encoder = std::make_unique<MultiTurnEncoderRTU>(config.encoder_ip.c_str(),
                                                            config.encoder_port,
                                                            config.encoder_slave);
        } else {
            encoder = std::make_unique<MultiTurnEncoderRTU>(config.encoder_dev.c_str(),
                                                            config.encoder_baud,
                                                            config.encoder_parity,
                                                            config.encoder_data_bit,
                                                            config.encoder_stop_bit,
                                                            config.encoder_slave);
        }
        if (!log_failure) encoder->setDisableCerr(true);
        if (!encoder->connect()) {
            encoder.reset();
            if (log_failure) {
                std::cerr << "[WARNING] encoder connect failed, continuing without encoder." << std::endl;
            }
            return false;
        }
        std::cout << "[INFO] encoder connected." << std::endl;
        return true;
    }

    bool try_init_hook_mqtt(bool log_failure = true) {
        if (config.hook_mqtt_device_id.empty()) {
            return false;
        }
        try {
            std::cout << "[INFO] hook mqtt connecting as " << config.hook_mqtt_device_id << "..." << std::endl;
            hook_mqtt = std::make_unique<HookWarningServer>(config.hook_mqtt_device_id); 
            std::cout << "[INFO] hook mqtt initialized, start() will print MQTT, battery(0x02),"
                      << " and work mode(0x06) response status."
                      << std::endl;
            // 模块对象初始化成功后，由 start() 基于 MQTT 连接与应答帧分别打印启动状态。
            return true;
        } catch (const std::exception& ex) {
            hook_mqtt.reset();
            if (log_failure) {
                std::cerr << "[WARNING] hook mqtt connect failed: " << ex.what()
                          << ", continuing without hook mqtt." << std::endl;
            }
            return false;
        } catch (...) {
            hook_mqtt.reset();
            if (log_failure) {
                std::cerr << "[WARNING] hook mqtt connect failed, continuing without hook mqtt." << std::endl;
            }
            return false;
        }
    }

    static bool should_retry(std::chrono::steady_clock::time_point& next_retry_at,
                             const std::chrono::steady_clock::time_point now) {
        if (now < next_retry_at) {
            return false;
        }
        next_retry_at = now + kReconnectInterval;
        return true;
    }

    void reconnect_missing_devices() {
        const auto now = std::chrono::steady_clock::now();

        if (!trolley && should_retry(next_trolley_reconnect_at_, now)) {
            if (!try_init_trolley(false)) {
                if (now - last_trolley_fail_log_ >= kFailLogInterval) {
                    std::cerr << "[WARNING] trolley still unreachable (retrying every "
                              << kReconnectInterval.count() << "s)..." << std::endl;
                    last_trolley_fail_log_ = now;
                }
            } else {
                std::cout << "[INFO] trolley reconnected successfully." << std::endl;
            }
        }
        if (!encoder && should_retry(next_encoder_reconnect_at_, now)) {
            if (!try_init_encoder(false)) {
                if (now - last_encoder_fail_log_ >= kFailLogInterval) {
                    std::cerr << "[WARNING] encoder still unreachable (retrying every "
                              << kReconnectInterval.count() << "s)..." << std::endl;
                    last_encoder_fail_log_ = now;
                }
            } else {
                std::cout << "[INFO] encoder reconnected successfully." << std::endl;
            }
        }
        if (!hook_mqtt && !config.hook_mqtt_device_id.empty() &&
            should_retry(next_hook_mqtt_reconnect_at_, now)) {
            try_init_hook_mqtt(false);
            if (hook_mqtt) {
                std::cout << "[INFO] hook mqtt reconnected successfully." << std::endl;
                hook_mqtt->start();
            }
        }
    }

    void poll_devices_once() {
        reconnect_missing_devices();
        bridge_.exchange_shared_memory(config, trolley.get(), hook_mqtt.get(), encoder.get());
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

    impl_->try_init_trolley();
    impl_->try_init_encoder();
    impl_->try_init_hook_mqtt();

    return {};
}

// ============================================================================
//  Device init / shutdown / has
// ============================================================================

ModbusManagerClient::Status ModbusManagerClient::initTrolley() {
    if (!impl_) impl_ = std::make_unique<Impl>();
    if (!impl_->config_loaded) return {false, "config not loaded"};
    bool ok = impl_->try_init_trolley();
    return {ok, ok ? "" : "trolley connect failed"};
}

ModbusManagerClient::Status ModbusManagerClient::initEncoder() {
    if (!impl_) impl_ = std::make_unique<Impl>();
    if (!impl_->config_loaded) return {false, "config not loaded"};
    bool ok = impl_->try_init_encoder();
    return {ok, ok ? "" : "encoder connect failed"};
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

    if (!impl_->hook_mqtt) {
        impl_->try_init_hook_mqtt();
    }
    if (impl_->hook_mqtt) {
        impl_->hook_mqtt->start();
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
    impl_->bridge_.getSignalTrolleyConnectFaultInfo().disconnect_all_slots();
    impl_->bridge_.getSignalTrolleyDevicesErrorInfo().disconnect_all_slots();
    impl_->bridge_.getSignalCraneState().disconnect_all_slots();
    impl_->bridge_.getSignalAlert().disconnect_all_slots();
    impl_->bridge_.getSignalPowerButton().disconnect_all_slots();

    if (impl_->poll_thread_.joinable()) {
        impl_->poll_thread_.join();
    }

    impl_->encoder.reset();
    impl_->trolley.reset();

    impl_->hook_mqtt.reset();

    return {};
}

// ============================================================================
//  Hook MQTT Device
// ============================================================================

ModbusManagerClient::Status ModbusManagerClient::initHookMqtt(const std::string& device_id) {
    if (device_id.empty()) {
        return {false, "hook mqtt device_id is empty"};
    }
    if (!impl_) {
        impl_ = std::make_unique<Impl>();
    }
    impl_->config.hook_mqtt_device_id = device_id;
    bool ok = impl_->try_init_hook_mqtt();
    return {ok, ok ? "" : "failed to init hook mqtt"};
}

ModbusManagerClient::Status ModbusManagerClient::resetHookMqtt() {
    if (!impl_) {
        return {};
    }
    impl_->hook_mqtt.reset();
    return {};
}

#define MM_HOOK_BOOL(sig, call, err)                                           \
ModbusManagerClient::Status ModbusManagerClient::sig {                         \
    if (!impl_ || !impl_->hook_mqtt) return {false, "hook mqtt not initialized"}; \
    bool ok = (call);                                                          \
    return {ok, ok ? "" : err};                                                \
}

#define MM_HOOK_VOID(sig, call)                                                \
ModbusManagerClient::Status ModbusManagerClient::sig {                         \
    if (!impl_ || !impl_->hook_mqtt) return {false, "hook mqtt not initialized"}; \
    call;                                                                      \
    return {};                                                                 \
}

MM_HOOK_BOOL(setHookMqttFlashLight(bool light_on, bool sound_7m, bool sound_3m, std::uint8_t volume),
             impl_->hook_mqtt->set_flash_light(light_on, sound_7m, sound_3m, volume),
             "no response from hook device")

MM_HOOK_BOOL(setHookMqttHeartbeatEnable(bool enable),
             impl_->hook_mqtt->set_heartbeat_enable(enable),
             "no response from hook device")

MM_HOOK_BOOL(setHookMqttSleepModeEnable(bool enable),
             impl_->hook_mqtt->set_sleep_mode_enable(enable),
             "no response from hook device")

ModbusManagerClient::Status ModbusManagerClient::setHookMqttTimeSchedule(
    std::uint8_t on_hour, std::uint8_t on_minute,
    std::uint8_t off_hour, std::uint8_t off_minute) {
    if (!impl_ || !impl_->hook_mqtt) {
        return {false, "hook mqtt not initialized"};
    }
    if (off_hour > 23 || on_hour > 23 || off_minute > 59 || on_minute > 59) {
        return {false, "invalid hook mqtt schedule time"};
    }
    bool ok = impl_->hook_mqtt->set_time_schedule(on_hour, on_minute, off_hour, off_minute);
    return {ok, ok ? "" : "no response from hook device"};
}

ModbusManagerClient::Status ModbusManagerClient::setHookMqttCurrentTime(
    std::uint8_t hour, std::uint8_t minute) {
    if (!impl_ || !impl_->hook_mqtt) {
        return {false, "hook mqtt not initialized"};
    }
    if (hour > 23 || minute > 59) {
        return {false, "invalid hook mqtt current time"};
    }
    bool ok = impl_->hook_mqtt->set_current_time(hour, minute);
    return {ok, ok ? "" : "no response from hook device"};
}

MM_HOOK_VOID(getHookMqttLightStatus(FlashLightCmdData& status),
             status = impl_->hook_mqtt->get_light_status())

MM_HOOK_VOID(getHookMqttBmsStatus(BmsStatusData& status),
             status = impl_->hook_mqtt->get_bms_status())

MM_HOOK_VOID(getHookMqttHeartbeatEnable(bool& enable),
             enable = impl_->hook_mqtt->get_heartbeat_enable())

MM_HOOK_VOID(getHookMqttWorkMode(std::uint8_t& mode),
             mode = impl_->hook_mqtt->get_work_mode())

MM_HOOK_VOID(getHookMqttSleepModeEnable(bool& enable),
             enable = impl_->hook_mqtt->get_sleep_mode_enable())

MM_HOOK_VOID(getHookMqttSleepSchedule(SleepScheduleData& status),
             status = impl_->hook_mqtt->get_sleep_schedule())

MM_HOOK_VOID(getHookMqttCurrentTime(CurrentTimeData& status),
             status = impl_->hook_mqtt->get_current_time())

MM_HOOK_VOID(getHookMqttErrorCode(std::uint8_t& error_code),
             error_code = impl_->hook_mqtt->get_error_code())

#undef MM_HOOK_BOOL
#undef MM_HOOK_VOID

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

MM_BOOL(setTrolleyPowerLaser(std::uint8_t value),
        trolley, "trolley",
        impl_->trolley->setPowerLaser(value),
        "setPowerLaser")

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

boost::signals2::connection ModbusManagerClient::connectTrolleyConnectFaultInfo(
    const std::function<void(ai_safety_common::TrolleyConnectFaultInfo)>& slot) {
    if (impl_) {
        return impl_->bridge_.getSignalTrolleyConnectFaultInfo().connect(slot);
    }
    return {};
}

boost::signals2::connection ModbusManagerClient::connectTrolleyDevicesErrorInfo(
    const std::function<void(ai_safety_common::TrolleyDevicesErrorInfo)>& slot) {
    if (impl_) {
        return impl_->bridge_.getSignalTrolleyDevicesErrorInfo().connect(slot);
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
