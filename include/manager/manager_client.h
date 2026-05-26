#pragma once

#include <cstdint>
#include <memory>
#include <string>

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

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
