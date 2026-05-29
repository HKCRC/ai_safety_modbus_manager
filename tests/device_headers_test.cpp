#include <chrono>
#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <thread>
#include <vector>

#include "bridge/shared_memory_bridge.h"
#include "config/modbus_config.h"
#include "devices/hookWarning.h"
#include "devices/multi_turn_encoder_rtu.h"
#include "devices/trolleyControl.h"

namespace {

struct LauncherOptions {
    std::string config_path;
};

const char* default_config_path() {
#ifdef MODBUS_MANAGER_DEFAULT_CONFIG
    return MODBUS_MANAGER_DEFAULT_CONFIG;
#else
    return "config/modbus_config.json";
#endif
}

std::string to_text(ai_safety_common::DeviceStatus::SolarChargeState value) {
    switch (value) {
        case ai_safety_common::DeviceStatus::SolarChargeState::Unknown: return "Unknown";
        case ai_safety_common::DeviceStatus::SolarChargeState::NotCharging: return "NotCharging";
        case ai_safety_common::DeviceStatus::SolarChargeState::Charging: return "Charging";
        case ai_safety_common::DeviceStatus::SolarChargeState::Fault: return "Fault";
    }
    return "Unknown";
}

std::string to_text(ai_safety_common::DeviceStatus::EquipmentState value) {
    switch (value) {
        case ai_safety_common::DeviceStatus::EquipmentState::Unknown: return "Unknown";
        case ai_safety_common::DeviceStatus::EquipmentState::Offline: return "Offline";
        case ai_safety_common::DeviceStatus::EquipmentState::Standby: return "Standby";
        case ai_safety_common::DeviceStatus::EquipmentState::Active: return "Active";
        case ai_safety_common::DeviceStatus::EquipmentState::Error: return "Error";
    }
    return "Unknown";
}

std::string to_text(ai_safety_common::FaultInfo::Category value) {
    switch (value) {
        case ai_safety_common::FaultInfo::Category::Unknown: return "Unknown";
        case ai_safety_common::FaultInfo::Category::CabBridgeFault: return "CabBridgeFault";
        case ai_safety_common::FaultInfo::Category::TrolleyBridgeFault: return "TrolleyBridgeFault";
        case ai_safety_common::FaultInfo::Category::TrolleyStm32Fault: return "TrolleyStm32Fault";
        case ai_safety_common::FaultInfo::Category::BmsCommunicationFault: return "BmsCommunicationFault";
        case ai_safety_common::FaultInfo::Category::MpptCommunicationFault: return "MpptCommunicationFault";
        case ai_safety_common::FaultInfo::Category::LaserCommunicationFault: return "LaserCommunicationFault";
        case ai_safety_common::FaultInfo::Category::CctvCommunicationFault: return "CctvCommunicationFault";
    }
    return "Unknown";
}

void print_shared_memory_preview(const ai_safety_common::DeviceStatus& status) {
    std::cout << "[提示] 将发送 DeviceStatus:\n"
              << "  timestamp=" << status.timestamp << '\n'
              << "  solarCharge=" << to_text(status.solarCharge) << '\n'
              << "  trolleyState=" << to_text(status.trolleyState) << '\n'
              << "  trolleyBattery.percent=" << static_cast<int>(status.trolleyBattery.percent) << '\n'
              << "  trolleyBattery.remainingMin=" << status.trolleyBattery.remainingMin << '\n'
              << "  trolleyBattery.isCharging=" << static_cast<int>(status.trolleyBattery.isCharging) << '\n'
              << "  trolleyBattery.chargingTimeMin=" << status.trolleyBattery.chargingTimeMin << '\n'
              << "  trolleyBattery.voltageV=" << status.trolleyBattery.voltageV << '\n'
              << "  trolleyBattery.currentA=" << status.trolleyBattery.currentA << '\n'
              << "  hookState=" << to_text(status.hookState) << '\n'
              << "  hookBattery.percent=" << static_cast<int>(status.hookBattery.percent) << '\n'
              << "  hookBattery.remainingMin=" << status.hookBattery.remainingMin << '\n'
              << "  hookBattery.isCharging=" << static_cast<int>(status.hookBattery.isCharging) << '\n'
              << "  hookBattery.chargingTimeMin=" << status.hookBattery.chargingTimeMin << '\n'
              << "  hookBattery.voltageV=" << status.hookBattery.voltageV << '\n'
              << "  hookBattery.currentA=" << status.hookBattery.currentA << '\n';
}

void print_shared_memory_preview(const ai_safety_common::FaultInfo& status) {
    std::cout << "[提示] 将发送 FaultInfo:\n"
              << "  timestamp=" << status.timestamp << '\n'
              << "  category=" << to_text(status.category) << '\n';
}

void print_shared_memory_preview(const ai_safety_common::CraneState& state) {
    std::cout << "[提示] 将发送 CraneState:\n"
              << "  timestamp=" << state.timestamp << '\n'
              << "  hookToTrolleyDistanceM=" << state.hookToTrolleyDistanceM << '\n'
              << "  groundToTrolleyDistanceM=" << state.groundToTrolleyDistanceM << '\n';
}

bool fail(const std::string& message) {
    std::cerr << "[失败] " << message << '\n';
    return false;
}

bool expect_ok(bool condition, const std::string& step) {
    if (!condition) {
        std::cerr << "[失败] " << step << '\n';
        return false;
    }
    std::cout << "[通过] " << step << '\n';
    return true;
}

void print_usage(const char* program) {
    std::cout
        << "用法:\n"
        << "  " << program << " [--config PATH]\n\n"
        << "说明:\n"
        << "  启动后进入单入口菜单，在菜单里选择设备和功能测试。\n";
}

bool parse_args(int argc, char** argv, LauncherOptions& options) {
    options.config_path = default_config_path();
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help") {
            print_usage(argv[0]);
            std::exit(EXIT_SUCCESS);
        }
        if (arg == "--config") {
            if (i + 1 >= argc) {
                return fail("--config 缺少路径参数");
            }
            options.config_path = argv[++i];
            continue;
        }
        options.config_path = arg;
    }
    return true;
}

std::string read_line(const std::string& prompt) {
    std::cout << prompt;
    std::cout.flush();

    std::string line;
    if (!std::getline(std::cin, line)) {
        return std::string();
    }
    return line;
}

int prompt_menu_choice(const std::string& title, const std::vector<std::string>& options) {
    while (true) {
        std::cout << "\n==== " << title << " ====\n";
        for (std::size_t i = 0; i < options.size(); ++i) {
            std::cout << "  " << i << ". " << options[i] << '\n';
        }

        const std::string line = read_line("请输入选项编号: ");
        if (line.empty()) {
            return 0;
        }

        try {
            const int choice = std::stoi(line);
            if (choice >= 0 && choice < static_cast<int>(options.size())) {
                return choice;
            }
        } catch (...) {
        }
        std::cout << "[警告] 无效输入，请重新输入。\n";
    }
}

bool prompt_yes_no(const std::string& prompt) {
    while (true) {
        const std::string line = read_line(prompt + " [y/n]: ");
        if (line == "y" || line == "Y") return true;
        if (line == "n" || line == "N" || line.empty()) return false;
        std::cout << "[警告] 请输入 y 或 n。\n";
    }
}

bool prompt_int_in_range(const std::string& label, int current, int min_value, int max_value, int& out_value) {
    while (true) {
        const std::string line = read_line(label + " [当前值=" + std::to_string(current) +
                                           ", 范围=" + std::to_string(min_value) + "~" +
                                           std::to_string(max_value) + ", 直接回车取消]: ");
        if (line.empty()) {
            std::cout << "[提示] 已取消写入\n";
            return false;
        }

        try {
            const int value = std::stoi(line);
            if (value < min_value || value > max_value) {
                std::cout << "[警告] 输入超出范围，请重新输入。\n";
                continue;
            }
            out_value = value;
            return true;
        } catch (...) {
            std::cout << "[警告] 无效输入，请输入整数。\n";
        }
    }
}

bool load_config(const std::string& path, ModbusConfig& config) {
    std::cout << "[提示] 正在加载配置文件: " << path << '\n';
    if (!config.loadFromFile(path)) {
        return fail("加载配置文件失败");
    }
    return true;
}

void print_config_summary(const ModbusConfig& config, const std::string& path) {
    std::cout << "\n==== 当前配置 ====\n"
              << "配置文件: " << path << '\n'
              << "trolley: " << config.trolley_ip << ':' << config.trolley_port
              << " slave=" << config.trolley_slave << '\n'
              << "hook:    " << config.hook_dev
              << " baud=" << config.hook_baud
              << " parity=" << config.hook_parity
              << " data=" << config.hook_data_bit
              << " stop=" << config.hook_stop_bit
              << " slave=" << config.hook_slave << '\n'
              << "encoder: " << config.encoder_dev
              << " baud=" << config.encoder_baud
              << " parity=" << config.encoder_parity
              << " data=" << config.encoder_data_bit
              << " stop=" << config.encoder_stop_bit
              << " slave=" << config.encoder_slave << '\n';
}

template <typename Device, typename Factory, typename Action>
bool with_connected_device(const std::string& name, Factory factory, Action action) {
    Device device = factory();
    if (!device.connect()) {
        return fail(name + " 连接失败");
    }

    bool ok = true;
    ok &= expect_ok(device.isConnected(), name + " 连接后 isConnected() 为真");
    if (ok) {
        ok &= action(device);
    }

    device.disconnect();
    ok &= expect_ok(!device.isConnected(), name + " 断开后 isConnected() 为假");
    return ok;
}

bool hook_read_and_print_status(HookWarning& hook) {
    bool ok = true;
    ok &= expect_ok(hook.refreshStatus(), "吊钩执行 refreshStatus()");
    if (!ok) {
        return false;
    }

    std::cout << std::fixed << std::setprecision(2)
              << "[提示] 吊钩状态: 电量=" << hook.get_battery_level_feedback()
              << "% 音量=" << hook.get_volume_feedback()
              << " 放电时间=" << hook.getDischargeTime()
              << " 工作模式=" << hook.getWorkmode()
              << " 充电状态=" << hook.getChargingStatus()
              << " 充电时间=" << hook.getChargeTime()
              << " 电压=" << hook.getVoltage()
              << " 电流=" << hook.getCurrent()
              << " 关机时间=" << hook.getShutdownTime()
              << " 开机时间=" << hook.getStartupTime()
              << " 错误码=" << hook.getErrorCode()
              << " STM时间=" << hook.getStmRtcTime()
              << " 待机使能=" << hook.getStandbyEnable()
              << " PC时间=" << hook.getPcRtcTime()
              << '\n';
    return true;
}

bool hook_test_basic(const ModbusConfig& config) {
    return with_connected_device<HookWarning>(
        "hook",
        [&]() {
            return HookWarning(config.hook_dev,
                               config.hook_baud,
                               config.hook_parity,
                               config.hook_data_bit,
                               config.hook_stop_bit,
                               config.hook_slave);
        },
        [&](HookWarning& hook) { return hook_read_and_print_status(hook); });
}

bool hook_test_volume(const ModbusConfig& config) {
    return with_connected_device<HookWarning>(
        "hook",
        [&]() {
            return HookWarning(config.hook_dev,
                               config.hook_baud,
                               config.hook_parity,
                               config.hook_data_bit,
                               config.hook_stop_bit,
                               config.hook_slave);
        },
        [&](HookWarning& hook) {
            if (!hook_read_and_print_status(hook)) {
                return false;
            }
            const uint16_t current_volume = hook.get_volume_feedback();
            return expect_ok(hook.setVolume(current_volume), "吊钩执行 setVolume(当前值)");
        });
}

bool hook_test_heartbeat(const ModbusConfig& config) {
    return with_connected_device<HookWarning>(
        "hook",
        [&]() {
            return HookWarning(config.hook_dev,
                               config.hook_baud,
                               config.hook_parity,
                               config.hook_data_bit,
                               config.hook_stop_bit,
                               config.hook_slave);
        },
        [&](HookWarning& hook) {
            bool ok = true;
            ok &= expect_ok(hook.sendHeartbeat(), "吊钩执行 sendHeartbeat()");
            uint16_t heartbeat_value = 0;
            ok &= expect_ok(hook.readHeartbeat(heartbeat_value), "吊钩执行 readHeartbeat()");
            if (ok) {
                std::cout << "[提示] 吊钩心跳值=" << heartbeat_value << '\n';
            }
            return ok;
        });
}

bool hook_test_rfid_getters(const ModbusConfig& config) {
    return with_connected_device<HookWarning>(
        "hook",
        [&]() {
            return HookWarning(config.hook_dev,
                               config.hook_baud,
                               config.hook_parity,
                               config.hook_data_bit,
                               config.hook_stop_bit,
                               config.hook_slave);
        },
        [&](HookWarning& hook) {
            if (!hook_read_and_print_status(hook)) {
                return false;
            }
            const uint16_t valid_index = hook.getValidIndex();
            const auto entries = hook.getValidRfidEntries();
            std::cout << "[提示] 吊钩有效 RFID 索引掩码=0x" << std::hex << valid_index << std::dec
                      << " 有效条目数=" << entries.size() << '\n';
            for (int i = 0; i < HookWarning::NUM_RFID_INDICES; ++i) {
                const bool valid = hook.isRfidValid(i);
                const RfidEntry entry = hook.getRfidEntry(i);
                const uint32_t uid = hook.getRfidUid(i);
                const uint8_t signal = hook.getRfidSignalStrength(i);
                const uint8_t battery = hook.getRfidBattery(i);
                if (valid) {
                    std::cout << "[提示] RFID[" << i << "] uid=" << uid
                              << " 信号强度=" << static_cast<int>(signal)
                              << " 电量=" << static_cast<int>(battery)
                              << " entry.uid=" << entry.uid << '\n';
                }
            }
            return true;
        });
}

bool hook_test_alert_sequence(const ModbusConfig& config, int signal_index, const std::string& name) {
    return with_connected_device<HookWarning>(
        "hook",
        [&]() {
            return HookWarning(config.hook_dev,
                               config.hook_baud,
                               config.hook_parity,
                               config.hook_data_bit,
                               config.hook_stop_bit,
                               config.hook_slave);
        },
        [&](HookWarning& hook) {
            bool ok = true;
            ok &= expect_ok(hook.slot_warning(signal_index), "吊钩执行 " + name);
            ok &= expect_ok(hook.slot_warning(0), "吊钩执行 slot_warning(0) 恢复");
            return ok;
        });
}

bool hook_test_run_thread(const ModbusConfig& config) {
    return with_connected_device<HookWarning>(
        "hook",
        [&]() {
            return HookWarning(config.hook_dev,
                               config.hook_baud,
                               config.hook_parity,
                               config.hook_data_bit,
                               config.hook_stop_bit,
                               config.hook_slave);
        },
        [&](HookWarning& hook) {
            const bool ok = expect_ok(hook.run(), "吊钩执行 run()");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            return ok;
        });
}

bool hook_test_all(const ModbusConfig& config) {
    bool ok = true;
    ok &= hook_test_basic(config);
    ok &= hook_test_volume(config);
    ok &= hook_test_heartbeat(config);
    ok &= hook_test_rfid_getters(config);
    if (prompt_yes_no("是否执行 Hook 3m/7m 声光测试?")) {
        ok &= hook_test_alert_sequence(config, 1, "slot_warning(1)");
        ok &= hook_test_alert_sequence(config, 2, "slot_warning(2)");
    }
    if (prompt_yes_no("是否执行 Hook run() 线程测试? 结束时可能等待几秒")) {
        ok &= hook_test_run_thread(config);
    }
    return ok;
}

bool trolley_read_and_print_status(TrolleyControl& trolley, TrolleyStatus& status) {
    if (!expect_ok(trolley.readStatus(status), "小车执行 readStatus()")) {
        return false;
    }
    std::cout << std::fixed << std::setprecision(2)
              << "[提示] 小车状态详情\n"
              << "  standby_mode_active=" << static_cast<int>(status.standby_mode_active) << '\n'
              << "  sleep_mode_active=" << static_cast<int>(status.sleep_mode_active) << '\n'
              << "  bms_read_ok=" << static_cast<int>(status.bms_read_ok) << '\n'
              << "  mppt_read_ok=" << static_cast<int>(status.mppt_read_ok) << '\n'
              << "  laser_1_read_ok=" << static_cast<int>(status.laser_1_read_ok) << '\n'
              << "  laser_2_read_ok=" << static_cast<int>(status.laser_2_read_ok) << '\n'
              << "  cctv_ping_ok=" << static_cast<int>(status.cctv_ping_ok) << '\n'
              << "  bms_charging=" << static_cast<int>(status.bms_charging) << '\n'
              << "  bridge_ping_ok=" << static_cast<int>(status.bridge_ping_ok) << '\n'
              << "  system_error_code=" << status.system_error_code << '\n'
              << "  battery_level=" << static_cast<int>(status.battery_level) << '\n'
              << "  discharge_time_valid=" << static_cast<int>(status.discharge_time_valid) << '\n'
              << "  discharge_time_min=" << status.discharge_time_min << '\n'
              << "  charge_time_valid=" << static_cast<int>(status.charge_time_valid) << '\n'
              << "  charge_time_min=" << status.charge_time_min << '\n'
              << "  battery_voltage_v=" << status.battery_voltage_v << '\n'
              << "  battery_current_a=" << status.battery_current_a << '\n'
              << "  laser_distance[0]=" << status.laser_distance[0] << '\n'
              << "  laser_distance[1]=" << status.laser_distance[1] << '\n'
              << "  mppt_charge_status=" << status.mppt_charge_status << '\n'
              << "  device_status=" << status.device_status << '\n';
    return true;
}

bool trolley_test_basic(const ModbusConfig& config) {
    return with_connected_device<TrolleyControl>(
        "trolley",
        [&]() { return TrolleyControl(config.trolley_ip, config.trolley_port, config.trolley_slave); },
        [&](TrolleyControl& trolley) {
            TrolleyStatus status{};
            return trolley_read_and_print_status(trolley, status);
        });
}

bool trolley_test_power_3v3(const ModbusConfig& config) {
    return with_connected_device<TrolleyControl>(
        "trolley",
        [&]() { return TrolleyControl(config.trolley_ip, config.trolley_port, config.trolley_slave); },
        [&](TrolleyControl& trolley) {
            TrolleyStatus status{};
            if (!trolley_read_and_print_status(trolley, status)) return false;
            int target = 0;
            if (!prompt_int_in_range("输入 3V3 电源目标值", 0, 0, 1, target)) return true;
            return expect_ok(trolley.setPower3v3(static_cast<std::uint8_t>(target)), "小车执行 setPower3v3(目标值)");
        });
}

bool trolley_test_power_5v(const ModbusConfig& config) {
    return with_connected_device<TrolleyControl>(
        "trolley",
        [&]() { return TrolleyControl(config.trolley_ip, config.trolley_port, config.trolley_slave); },
        [&](TrolleyControl& trolley) {
            TrolleyStatus status{};
            if (!trolley_read_and_print_status(trolley, status)) return false;
            int target = 0;
            if (!prompt_int_in_range("输入 5V 电源目标值", 0, 0, 1, target)) return true;
            return expect_ok(trolley.setPower5v(static_cast<std::uint8_t>(target)), "小车执行 setPower5v(目标值)");
        });
}

bool trolley_test_power_cctv(const ModbusConfig& config) {
    return with_connected_device<TrolleyControl>(
        "trolley",
        [&]() { return TrolleyControl(config.trolley_ip, config.trolley_port, config.trolley_slave); },
        [&](TrolleyControl& trolley) {
            TrolleyStatus status{};
            if (!trolley_read_and_print_status(trolley, status)) return false;
            int target = 0;
            if (!prompt_int_in_range("输入 CCTV 电源目标值", 0, 0, 1, target)) return true;
            return expect_ok(trolley.setPowerCctv(static_cast<std::uint8_t>(target)), "小车执行 setPowerCctv(目标值)");
        });
}

bool trolley_test_power_4g(const ModbusConfig& config) {
    return with_connected_device<TrolleyControl>(
        "trolley",
        [&]() { return TrolleyControl(config.trolley_ip, config.trolley_port, config.trolley_slave); },
        [&](TrolleyControl& trolley) {
            TrolleyStatus status{};
            if (!trolley_read_and_print_status(trolley, status)) return false;
            int target = 0;
            if (!prompt_int_in_range("输入 4G 电源目标值", 0, 0, 1, target)) return true;
            return expect_ok(trolley.setPower4g(static_cast<std::uint8_t>(target)), "小车执行 setPower4g(目标值)");
        });
}

bool trolley_test_standby_enable(const ModbusConfig& config) {
    return with_connected_device<TrolleyControl>(
        "trolley",
        [&]() { return TrolleyControl(config.trolley_ip, config.trolley_port, config.trolley_slave); },
        [&](TrolleyControl& trolley) {
            TrolleyStatus status{};
            if (!trolley_read_and_print_status(trolley, status)) return false;
            int target = 0;
            if (!prompt_int_in_range("输入待机使能目标值", status.standby_mode_active, 0, 1, target)) return true;
            return expect_ok(trolley.setStandbyEnable(static_cast<std::uint8_t>(target)),
                             "小车执行 setStandbyEnable(目标值)");
        });
}

bool trolley_test_standby_power_mode(const ModbusConfig& config) {
    return with_connected_device<TrolleyControl>(
        "trolley",
        [&]() { return TrolleyControl(config.trolley_ip, config.trolley_port, config.trolley_slave); },
        [&](TrolleyControl& trolley) {
            TrolleyStatus status{};
            if (!trolley_read_and_print_status(trolley, status)) return false;
            int target = 0;
            if (!prompt_int_in_range("输入待机电源模式目标值", status.standby_mode_active, 0, 1, target)) return true;
            return expect_ok(trolley.setStandbyPowerMode(static_cast<std::uint8_t>(target)),
                             "小车执行 setStandbyPowerMode(目标值)");
        });
}

bool trolley_test_work_mode(const ModbusConfig& config) {
    return with_connected_device<TrolleyControl>(
        "trolley",
        [&]() { return TrolleyControl(config.trolley_ip, config.trolley_port, config.trolley_slave); },
        [&](TrolleyControl& trolley) {
            TrolleyStatus status{};
            if (!trolley_read_and_print_status(trolley, status)) return false;
            int target = 0;
            if (!prompt_int_in_range("输入工作模式目标值", 0, 0, 65535, target)) return true;
            return expect_ok(trolley.setWorkMode(static_cast<uint16_t>(target)), "小车执行 setWorkMode(目标值)");
        });
}

bool trolley_test_rtc(const ModbusConfig& config) {
    return with_connected_device<TrolleyControl>(
        "trolley",
        [&]() { return TrolleyControl(config.trolley_ip, config.trolley_port, config.trolley_slave); },
        [&](TrolleyControl& trolley) {
            TrolleyStatus status{};
            if (!trolley_read_and_print_status(trolley, status)) return false;
            int hour = 0;
            int minute = 0;
            if (!prompt_int_in_range("输入 RTC 小时", 0, 0, 23, hour)) return true;
            if (!prompt_int_in_range("输入 RTC 分钟", 0, 0, 59, minute)) return true;
            return expect_ok(trolley.setRtcTime(static_cast<uint16_t>(hour), static_cast<uint16_t>(minute)),
                             "小车执行 setRtcTime(目标值)");
        });
}

bool trolley_test_startup_time(const ModbusConfig& config) {
    return with_connected_device<TrolleyControl>(
        "trolley",
        [&]() { return TrolleyControl(config.trolley_ip, config.trolley_port, config.trolley_slave); },
        [&](TrolleyControl& trolley) {
            TrolleyStatus status{};
            if (!trolley_read_and_print_status(trolley, status)) return false;
            int target = 0;
            if (!prompt_int_in_range("输入开机时间目标值", 0, 0, 65535, target)) return true;
            return expect_ok(trolley.setStartupTime(static_cast<uint16_t>(target)), "小车执行 setStartupTime(目标值)");
        });
}

bool trolley_test_shutdown_time(const ModbusConfig& config) {
    return with_connected_device<TrolleyControl>(
        "trolley",
        [&]() { return TrolleyControl(config.trolley_ip, config.trolley_port, config.trolley_slave); },
        [&](TrolleyControl& trolley) {
            TrolleyStatus status{};
            if (!trolley_read_and_print_status(trolley, status)) return false;
            int target = 0;
            if (!prompt_int_in_range("输入关机时间目标值", 0, 0, 65535, target)) return true;
            return expect_ok(trolley.setShutdownTime(static_cast<uint16_t>(target)), "小车执行 setShutdownTime(目标值)");
        });
}

bool trolley_test_reboot(const ModbusConfig& config) {
    if (!prompt_yes_no("确认触发小车 reboot? 这是高风险操作")) {
        std::cout << "[提示] 已取消小车 reboot 测试\n";
        return true;
    }
    return with_connected_device<TrolleyControl>(
        "trolley",
        [&]() { return TrolleyControl(config.trolley_ip, config.trolley_port, config.trolley_slave); },
        [&](TrolleyControl& trolley) { return expect_ok(trolley.triggerReboot(), "小车执行 triggerReboot()"); });
}

bool trolley_test_all(const ModbusConfig& config) {
    bool ok = true;
    ok &= trolley_test_basic(config);
    ok &= trolley_test_power_3v3(config);
    ok &= trolley_test_power_5v(config);
    ok &= trolley_test_power_cctv(config);
    ok &= trolley_test_power_4g(config);
    ok &= trolley_test_standby_enable(config);
    ok &= trolley_test_standby_power_mode(config);
    ok &= trolley_test_work_mode(config);
    ok &= trolley_test_rtc(config);
    ok &= trolley_test_startup_time(config);
    ok &= trolley_test_shutdown_time(config);
    ok &= trolley_test_reboot(config);
    return ok;
}

bool encoder_read_position(MultiTurnEncoderRTU& encoder, int32_t& position) {
    return expect_ok(encoder.readEncoderPosition(position), "编码器执行 readEncoderPosition()");
}

bool encoder_read_settings(MultiTurnEncoderRTU& encoder, MultiTurnEncoderRTU::EncoderSettings& settings) {
    return expect_ok(encoder.readEncoderSettings(settings), "编码器执行 readEncoderSettings()");
}

bool encoder_test_basic(const ModbusConfig& config) {
    return with_connected_device<MultiTurnEncoderRTU>(
        "encoder",
        [&]() {
            return MultiTurnEncoderRTU(config.encoder_dev.c_str(),
                                       config.encoder_baud,
                                       config.encoder_parity,
                                       config.encoder_data_bit,
                                       config.encoder_stop_bit,
                                       config.encoder_slave);
        },
        [&](MultiTurnEncoderRTU& encoder) {
            int32_t position = 0;
            MultiTurnEncoderRTU::EncoderSettings settings{};
            bool ok = true;
            ok &= encoder_read_position(encoder, position);
            ok &= encoder_read_settings(encoder, settings);
            if (ok) {
                std::cout << "[提示] 编码器位置=" << position
                          << " 地址=" << settings.deviceAddress
                          << " 波特率=" << static_cast<int>(settings.baudRate)
                          << " 计数方向=" << static_cast<int>(settings.countingDirection)
                          << " 校验位=" << static_cast<int>(settings.parityCheck)
                          << '\n';
            }
            return ok;
        });
}

bool encoder_test_turns(const ModbusConfig& config) {
    return with_connected_device<MultiTurnEncoderRTU>(
        "encoder",
        [&]() {
            return MultiTurnEncoderRTU(config.encoder_dev.c_str(),
                                       config.encoder_baud,
                                       config.encoder_parity,
                                       config.encoder_data_bit,
                                       config.encoder_stop_bit,
                                       config.encoder_slave);
        },
        [&](MultiTurnEncoderRTU& encoder) {
            double turns = 0.0;
            double timestamp = 0.0;
            double duration = 0.0;
            const bool ok = expect_ok(encoder.readEncoderNumberOfTurns(turns, timestamp, duration),
                                      "编码器执行 readEncoderNumberOfTurns()");
            if (ok) {
                std::cout << "[提示] 编码器圈数=" << turns
                          << " 时间戳=" << timestamp
                          << " 持续时间=" << duration << '\n';
            }
            return ok;
        });
}

bool encoder_test_write_position(const ModbusConfig& config) {
    return with_connected_device<MultiTurnEncoderRTU>(
        "encoder",
        [&]() {
            return MultiTurnEncoderRTU(config.encoder_dev.c_str(),
                                       config.encoder_baud,
                                       config.encoder_parity,
                                       config.encoder_data_bit,
                                       config.encoder_stop_bit,
                                       config.encoder_slave);
        },
        [&](MultiTurnEncoderRTU& encoder) {
            int32_t position = 0;
            if (!encoder_read_position(encoder, position)) return false;
            return expect_ok(encoder.writeEncoderPosition(position), "编码器执行 writeEncoderPosition(当前值)");
        });
}

bool encoder_test_raw_settings(const ModbusConfig& config) {
    return with_connected_device<MultiTurnEncoderRTU>(
        "encoder",
        [&]() {
            return MultiTurnEncoderRTU(config.encoder_dev.c_str(),
                                       config.encoder_baud,
                                       config.encoder_parity,
                                       config.encoder_data_bit,
                                       config.encoder_stop_bit,
                                       config.encoder_slave);
        },
        [&](MultiTurnEncoderRTU& encoder) {
            uint16_t regs[4] = {0};
            const bool ok = expect_ok(encoder.readRawSettings(regs), "编码器执行 readRawSettings()");
            if (ok) {
                std::cout << "[提示] 原始设置寄存器: [" << regs[0] << ", " << regs[1] << ", " << regs[2]
                          << ", " << regs[3] << "]\n";
            }
            return ok;
        });
}

bool encoder_test_string_settings(const ModbusConfig& config) {
    return with_connected_device<MultiTurnEncoderRTU>(
        "encoder",
        [&]() {
            return MultiTurnEncoderRTU(config.encoder_dev.c_str(),
                                       config.encoder_baud,
                                       config.encoder_parity,
                                       config.encoder_data_bit,
                                       config.encoder_stop_bit,
                                       config.encoder_slave);
        },
        [&](MultiTurnEncoderRTU& encoder) {
            const auto settings = encoder.getEncoderSettings();
            const bool ok = (settings.deviceAddress != "Error");
            if (!expect_ok(ok, "编码器执行 getEncoderSettings()")) {
                return false;
            }
            std::cout << "[提示] 编码器格式化设置: 地址=" << settings.deviceAddress
                      << " 波特率=" << settings.baudRate
                      << " 计数方向=" << settings.countingDirection
                      << " 校验位=" << settings.parityCheck << '\n';
            return true;
        });
}

bool encoder_test_write_device_address(const ModbusConfig& config) {
    return with_connected_device<MultiTurnEncoderRTU>(
        "encoder",
        [&]() {
            return MultiTurnEncoderRTU(config.encoder_dev.c_str(),
                                       config.encoder_baud,
                                       config.encoder_parity,
                                       config.encoder_data_bit,
                                       config.encoder_stop_bit,
                                       config.encoder_slave);
        },
        [&](MultiTurnEncoderRTU& encoder) {
            MultiTurnEncoderRTU::EncoderSettings settings{};
            if (!encoder_read_settings(encoder, settings)) return false;
            return expect_ok(encoder.write485DeviceAddress(settings.deviceAddress),
                             "编码器执行 write485DeviceAddress(当前值)");
        });
}

bool encoder_test_write_baud(const ModbusConfig& config) {
    return with_connected_device<MultiTurnEncoderRTU>(
        "encoder",
        [&]() {
            return MultiTurnEncoderRTU(config.encoder_dev.c_str(),
                                       config.encoder_baud,
                                       config.encoder_parity,
                                       config.encoder_data_bit,
                                       config.encoder_stop_bit,
                                       config.encoder_slave);
        },
        [&](MultiTurnEncoderRTU& encoder) {
            MultiTurnEncoderRTU::EncoderSettings settings{};
            if (!encoder_read_settings(encoder, settings)) return false;
            return expect_ok(encoder.writeBaudRate(settings.baudRate), "编码器执行 writeBaudRate(当前值)");
        });
}

bool encoder_test_write_direction(const ModbusConfig& config) {
    return with_connected_device<MultiTurnEncoderRTU>(
        "encoder",
        [&]() {
            return MultiTurnEncoderRTU(config.encoder_dev.c_str(),
                                       config.encoder_baud,
                                       config.encoder_parity,
                                       config.encoder_data_bit,
                                       config.encoder_stop_bit,
                                       config.encoder_slave);
        },
        [&](MultiTurnEncoderRTU& encoder) {
            MultiTurnEncoderRTU::EncoderSettings settings{};
            if (!encoder_read_settings(encoder, settings)) return false;
            return expect_ok(encoder.writeCountingDirection(settings.countingDirection),
                             "编码器执行 writeCountingDirection(当前值)");
        });
}

bool encoder_test_write_parity(const ModbusConfig& config) {
    return with_connected_device<MultiTurnEncoderRTU>(
        "encoder",
        [&]() {
            return MultiTurnEncoderRTU(config.encoder_dev.c_str(),
                                       config.encoder_baud,
                                       config.encoder_parity,
                                       config.encoder_data_bit,
                                       config.encoder_stop_bit,
                                       config.encoder_slave);
        },
        [&](MultiTurnEncoderRTU& encoder) {
            MultiTurnEncoderRTU::EncoderSettings settings{};
            if (!encoder_read_settings(encoder, settings)) return false;
            return expect_ok(encoder.writeParityCheck(settings.parityCheck),
                             "编码器执行 writeParityCheck(当前值)");
        });
}

bool encoder_test_run_once_and_data(const ModbusConfig& config) {
    return with_connected_device<MultiTurnEncoderRTU>(
        "encoder",
        [&]() {
            return MultiTurnEncoderRTU(config.encoder_dev.c_str(),
                                       config.encoder_baud,
                                       config.encoder_parity,
                                       config.encoder_data_bit,
                                       config.encoder_stop_bit,
                                       config.encoder_slave);
        },
        [&](MultiTurnEncoderRTU& encoder) {
            encoder.run_once();
            const StampedDouble data = encoder.getData();
            const double timestamp = encoder.getDataTimestamp();
            const auto encoder_data = encoder.getEncoderData();
            std::cout << "[提示] 编码器 getRunLoopSleepMs=" << encoder.getRunLoopSleepMs() << '\n'
                      << "[提示] 编码器 getData(): 时间戳=" << data.timestamp
                      << " 方差=" << data.time_variance
                      << " 数值=" << data.value << '\n'
                      << "[提示] 编码器 getDataTimestamp()=" << timestamp << '\n'
                      << "[提示] 编码器 getEncoderData(): 时间戳=" << encoder_data.timestamp
                      << " 方差=" << encoder_data.time_variance
                      << " 数值=" << encoder_data.value
                      << " 速度=" << encoder_data.velocity << '\n';
            return true;
        });
}

bool encoder_test_update_and_velocity(const ModbusConfig& config) {
    return with_connected_device<MultiTurnEncoderRTU>(
        "encoder",
        [&]() {
            return MultiTurnEncoderRTU(config.encoder_dev.c_str(),
                                       config.encoder_baud,
                                       config.encoder_parity,
                                       config.encoder_data_bit,
                                       config.encoder_stop_bit,
                                       config.encoder_slave);
        },
        [&](MultiTurnEncoderRTU& encoder) {
            double turns = 0.0;
            double timestamp = 0.0;
            double duration = 0.0;
            if (!expect_ok(encoder.readEncoderNumberOfTurns(turns, timestamp, duration),
                           "编码器在 updateEncoderData() 前执行 readEncoderNumberOfTurns()")) {
                return false;
            }
            for (int i = 0; i < 120; ++i) {
                encoder.updateEncoderData(turns + static_cast<double>(i) * 0.001,
                                          timestamp + static_cast<double>(i) * 0.01,
                                          duration);
            }
            const double velocity = encoder.computeVelocity();
            const auto encoder_data = encoder.getEncoderData();
            std::cout << "[提示] 编码器计算速度=" << velocity
                      << " 最新数值=" << encoder_data.value
                      << " 最新速度=" << encoder_data.velocity << '\n';
            return true;
        });
}

bool encoder_test_run_thread(const ModbusConfig& config) {
    return with_connected_device<MultiTurnEncoderRTU>(
        "encoder",
        [&]() {
            return MultiTurnEncoderRTU(config.encoder_dev.c_str(),
                                       config.encoder_baud,
                                       config.encoder_parity,
                                       config.encoder_data_bit,
                                       config.encoder_stop_bit,
                                       config.encoder_slave);
        },
        [&](MultiTurnEncoderRTU& encoder) {
            encoder.run();
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            encoder.stop();
            return expect_ok(true, "编码器执行 run()/stop()");
        });
}

bool encoder_test_all(const ModbusConfig& config) {
    bool ok = true;
    ok &= encoder_test_basic(config);
    ok &= encoder_test_turns(config);
    ok &= encoder_test_write_position(config);
    ok &= encoder_test_raw_settings(config);
    ok &= encoder_test_string_settings(config);
    ok &= encoder_test_write_device_address(config);
    ok &= encoder_test_write_baud(config);
    ok &= encoder_test_write_direction(config);
    ok &= encoder_test_write_parity(config);
    ok &= encoder_test_run_once_and_data(config);
    ok &= encoder_test_update_and_velocity(config);
    if (prompt_yes_no("是否执行 encoder run/stop 线程测试?")) {
        ok &= encoder_test_run_thread(config);
    }
    return ok;
}

bool shared_memory_preview_test(const ModbusConfig& config) {
    std::cout << "[提示] 开始共享内存发送预览测试，只打印桥接层准备发送的数据，不会真正对外发送。\n";

    TrolleyControl trolley(config.trolley_ip, config.trolley_port, config.trolley_slave);
    if (!trolley.connect()) {
        return fail("共享内存预览测试时，小车连接失败");
    }

    HookWarning hook(config.hook_dev,
                     config.hook_baud,
                     config.hook_parity,
                     config.hook_data_bit,
                     config.hook_stop_bit,
                     config.hook_slave);
    if (!hook.connect()) {
        trolley.disconnect();
        return fail("共享内存预览测试时，吊钩连接失败");
    }

    MultiTurnEncoderRTU encoder(config.encoder_dev.c_str(),
                                config.encoder_baud,
                                config.encoder_parity,
                                config.encoder_data_bit,
                                config.encoder_stop_bit,
                                config.encoder_slave);
    MultiTurnEncoderRTU* encoder_ptr = nullptr;
    if (encoder.connect()) {
        encoder_ptr = &encoder;
        std::cout << "[提示] 编码器已连接，将一起参与共享内存预览。\n";
    } else {
        std::cout << "[警告] 编码器连接失败，本次共享内存预览将忽略编码器相关字段。\n";
    }

    SharedMemoryBridge bridge{};
    bool got_device_status = false;
    bool got_trolley_offline = false;
    bool got_crane_state = false;

    const auto device_conn = bridge.getSignalDeviceStatus().connect([&](ai_safety_common::DeviceStatus status) {
        got_device_status = true;
        print_shared_memory_preview(status);
    });
    const auto trolley_offline_conn =
        bridge.getSignalFaultInfo().connect([&](ai_safety_common::FaultInfo status) {
            got_trolley_offline = true;
            print_shared_memory_preview(status);
        });
    const auto crane_conn = bridge.getSignalCraneState().connect([&](ai_safety_common::CraneState state_value) {
        got_crane_state = true;
        print_shared_memory_preview(state_value);
    });
    const auto alert_conn = bridge.getSignalAlert().connect([&](ai_safety_common::AlertMessage& alert_message) {
        alert_message.Enable3Alert = false;
        alert_message.Enable7Alert = false;
    });
    const auto power_conn = bridge.getSignalPowerButton().connect([&](std::uint8_t& power_command) { power_command = 0; });

    bridge.exchange_shared_memory(config, trolley, hook, encoder_ptr);

    device_conn.disconnect();
    trolley_offline_conn.disconnect();
    crane_conn.disconnect();
    alert_conn.disconnect();
    power_conn.disconnect();

    if (encoder_ptr != nullptr) {
        encoder.disconnect();
    }
    hook.disconnect();
    trolley.disconnect();

    bool ok = true;
    ok &= expect_ok(got_device_status, "共享内存预览已生成 DeviceStatus");
    ok &= expect_ok(got_trolley_offline, "共享内存预览已生成 FaultInfo");
    ok &= expect_ok(got_crane_state, "共享内存预览已生成 CraneState");
    return ok;
}

void run_hook_menu(const ModbusConfig& config) {
    while (true) {
        const int choice = prompt_menu_choice(
            "HookWarning 功能测试",
            {"返回上一级",
             "基础连接/状态读取",
             "音量写回当前值",
             "心跳写读测试",
             "RFID 与状态 getter 测试",
             "3 米报警开关测试",
             "7 米报警开关测试",
             "停止所有报警测试",
             "run() 线程测试",
             "全功能测试"});

        bool ok = true;
        switch (choice) {
            case 0: return;
            case 1: ok = hook_test_basic(config); break;
            case 2: ok = hook_test_volume(config); break;
            case 3: ok = hook_test_heartbeat(config); break;
            case 4: ok = hook_test_rfid_getters(config); break;
            case 5:
                if (prompt_yes_no("确认执行 Hook 3 米声光测试?")) {
                    ok = hook_test_alert_sequence(config, 1, "slot_warning(1)");
                }
                break;
            case 6:
                if (prompt_yes_no("确认执行 Hook 7 米声光测试?")) {
                    ok = hook_test_alert_sequence(config, 2, "slot_warning(2)");
                }
                break;
            case 7:
                ok = hook_test_alert_sequence(config, 0, "slot_warning(0)");
                break;
            case 8:
                if (prompt_yes_no("run() 测试结束时可能等待几秒，是否继续?")) {
                    ok = hook_test_run_thread(config);
                }
                break;
            case 9: ok = hook_test_all(config); break;
            default: ok = false; break;
        }
        std::cout << (ok ? "[提示] Hook 菜单测试完成\n" : "[提示] Hook 菜单测试失败\n");
    }
}

void run_trolley_menu(const ModbusConfig& config) {
    while (true) {
        const int choice = prompt_menu_choice(
            "TrolleyControl 功能测试",
            {"返回上一级",
             "基础连接/状态读取",
             "设置 3V3 电源",
             "设置 5V 电源",
             "设置 CCTV 电源",
             "设置 4G 电源",
             "设置待机使能",
             "设置待机电源模式",
             "设置工作模式",
             "设置 RTC",
             "设置开机时间",
             "设置关机时间",
             "Reboot 测试",
             "全功能测试"});

        bool ok = true;
        switch (choice) {
            case 0: return;
            case 1: ok = trolley_test_basic(config); break;
            case 2: ok = trolley_test_power_3v3(config); break;
            case 3: ok = trolley_test_power_5v(config); break;
            case 4: ok = trolley_test_power_cctv(config); break;
            case 5: ok = trolley_test_power_4g(config); break;
            case 6: ok = trolley_test_standby_enable(config); break;
            case 7: ok = trolley_test_standby_power_mode(config); break;
            case 8: ok = trolley_test_work_mode(config); break;
            case 9: ok = trolley_test_rtc(config); break;
            case 10: ok = trolley_test_startup_time(config); break;
            case 11: ok = trolley_test_shutdown_time(config); break;
            case 12: ok = trolley_test_reboot(config); break;
            case 13: ok = trolley_test_all(config); break;
            default: ok = false; break;
        }
        std::cout << (ok ? "[提示] Trolley 菜单测试完成\n" : "[提示] Trolley 菜单测试失败\n");
    }
}

void run_encoder_menu(const ModbusConfig& config) {
    while (true) {
        const int choice = prompt_menu_choice(
            "MultiTurnEncoderRTU 功能测试",
            {"返回上一级",
             "基础连接/位置/设置读取",
             "圈数与时间读取",
             "写回当前位置",
             "读取原始设置寄存器",
             "读取格式化设置",
             "写回当前设备地址",
             "写回当前波特率",
             "写回当前计数方向",
             "写回当前校验位",
             "run_once / getData / getEncoderData 测试",
             "updateEncoderData / computeVelocity 测试",
             "run / stop 线程测试",
             "全功能测试"});

        bool ok = true;
        switch (choice) {
            case 0: return;
            case 1: ok = encoder_test_basic(config); break;
            case 2: ok = encoder_test_turns(config); break;
            case 3: ok = encoder_test_write_position(config); break;
            case 4: ok = encoder_test_raw_settings(config); break;
            case 5: ok = encoder_test_string_settings(config); break;
            case 6: ok = encoder_test_write_device_address(config); break;
            case 7: ok = encoder_test_write_baud(config); break;
            case 8: ok = encoder_test_write_direction(config); break;
            case 9: ok = encoder_test_write_parity(config); break;
            case 10: ok = encoder_test_run_once_and_data(config); break;
            case 11: ok = encoder_test_update_and_velocity(config); break;
            case 12:
                if (prompt_yes_no("确认执行 encoder run/stop 线程测试?")) {
                    ok = encoder_test_run_thread(config);
                }
                break;
            case 13: ok = encoder_test_all(config); break;
            default: ok = false; break;
        }
        std::cout << (ok ? "[提示] Encoder 菜单测试完成\n" : "[提示] Encoder 菜单测试失败\n");
    }
}

void run_all_devices_menu(const ModbusConfig& config) {
    bool ok = true;
    ok &= hook_test_all(config);
    ok &= trolley_test_all(config);
    ok &= encoder_test_all(config);
    std::cout << (ok ? "[提示] 全部设备测试完成\n" : "[提示] 全部设备测试存在失败\n");
}

}  // namespace

int main(int argc, char** argv) {
    try {
        LauncherOptions options;
        if (!parse_args(argc, argv, options)) {
            print_usage(argv[0]);
            return EXIT_FAILURE;
        }

        ModbusConfig config;
        if (!load_config(options.config_path, config)) {
            return EXIT_FAILURE;
        }

        while (true) {
            const int choice = prompt_menu_choice(
                "硬件测试启动菜单",
                {"退出",
                 "查看当前配置",
                 "切换配置文件",
                 "共享内存发送预览测试",
                 "HookWarning 功能测试",
                 "TrolleyControl 功能测试",
                 "MultiTurnEncoderRTU 功能测试",
                 "全部设备顺序测试"});

            switch (choice) {
                case 0:
                    std::cout << "[提示] 退出测试程序\n";
                    return EXIT_SUCCESS;
                case 1:
                    print_config_summary(config, options.config_path);
                    break;
                case 2: {
                    const std::string new_path = read_line("输入新的配置文件路径: ");
                    if (!new_path.empty()) {
                        ModbusConfig new_config;
                        if (load_config(new_path, new_config)) {
                            config = new_config;
                            options.config_path = new_path;
                            std::cout << "[提示] 配置文件切换成功\n";
                        } else {
                            std::cout << "[警告] 配置文件切换失败，保留原配置\n";
                        }
                    }
                    break;
                }
                case 3:
                    shared_memory_preview_test(config);
                    break;
                case 4:
                    run_hook_menu(config);
                    break;
                case 5:
                    run_trolley_menu(config);
                    break;
                case 6:
                    run_encoder_menu(config);
                    break;
                case 7:
                    run_all_devices_menu(config);
                    break;
                default:
                    break;
            }
        }
    } catch (const std::exception& ex) {
        std::cerr << "[失败] 未处理异常: " << ex.what() << '\n';
    } catch (...) {
        std::cerr << "[失败] 未知异常\n";
    }
    return EXIT_FAILURE;
}
