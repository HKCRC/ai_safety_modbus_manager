#include "manager/manager_client.h"

#include <atomic>
#include <chrono>
#include <iostream>
#include <thread>
#include <utility>

#include "devices/hookWarning.h"
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

    std::thread worker;
    std::atomic<bool> running{false};
    SharedMemoryBridgeState shared_memory_state{};

    void publish_once() {
        exchange_shared_memory(config, *trolley, *hook, encoder.get(), shared_memory_state);
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
    return {};
}
