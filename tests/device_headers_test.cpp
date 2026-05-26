#include <chrono>
#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <thread>
#include <vector>

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

bool fail(const std::string& message) {
    std::cerr << "[FAIL] " << message << '\n';
    return false;
}

bool expect_ok(bool condition, const std::string& step) {
    if (!condition) {
        std::cerr << "[FAIL] " << step << '\n';
        return false;
    }
    std::cout << "[PASS] " << step << '\n';
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
        std::cout << "[WARN] 无效输入，请重新输入。\n";
    }
}

bool prompt_yes_no(const std::string& prompt) {
    while (true) {
        const std::string line = read_line(prompt + " [y/n]: ");
        if (line == "y" || line == "Y") return true;
        if (line == "n" || line == "N" || line.empty()) return false;
        std::cout << "[WARN] 请输入 y 或 n。\n";
    }
}

bool prompt_int_in_range(const std::string& label, int current, int min_value, int max_value, int& out_value) {
    while (true) {
        const std::string line = read_line(label + " [当前值=" + std::to_string(current) +
                                           ", 范围=" + std::to_string(min_value) + "~" +
                                           std::to_string(max_value) + ", 直接回车取消]: ");
        if (line.empty()) {
            std::cout << "[INFO] 已取消写入\n";
            return false;
        }

        try {
            const int value = std::stoi(line);
            if (value < min_value || value > max_value) {
                std::cout << "[WARN] 输入超出范围，请重新输入。\n";
                continue;
            }
            out_value = value;
            return true;
        } catch (...) {
            std::cout << "[WARN] 无效输入，请输入整数。\n";
        }
    }
}

bool load_config(const std::string& path, ModbusConfig& config) {
    std::cout << "[INFO] loading config: " << path << '\n';
    if (!config.loadFromFile(path)) {
        return fail("failed to load config file");
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
        return fail(name + " connect failed");
    }

    bool ok = true;
    ok &= expect_ok(device.isConnected(), name + " isConnected() after connect");
    if (ok) {
        ok &= action(device);
    }

    device.disconnect();
    ok &= expect_ok(!device.isConnected(), name + " isConnected() after disconnect");
    return ok;
}

bool hook_read_and_print_status(HookWarning& hook) {
    bool ok = true;
    ok &= expect_ok(hook.refreshStatus(), "hook refreshStatus()");
    if (!ok) {
        return false;
    }

    std::cout << std::fixed << std::setprecision(2)
              << "[INFO] hook battery=" << hook.get_battery_level_feedback()
              << "% volume=" << hook.get_volume_feedback()
              << " discharge=" << hook.getDischargeTime()
              << " workmode=" << hook.getWorkmode()
              << " charging=" << hook.getChargingStatus()
              << " charge_time=" << hook.getChargeTime()
              << " voltage=" << hook.getVoltage()
              << " current=" << hook.getCurrent()
              << " shutdown=" << hook.getShutdownTime()
              << " startup=" << hook.getStartupTime()
              << " error=" << hook.getErrorCode()
              << " stm_rtc=" << hook.getStmRtcTime()
              << " standby_enable=" << hook.getStandbyEnable()
              << " pc_rtc=" << hook.getPcRtcTime()
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
            return expect_ok(hook.setVolume(current_volume), "hook setVolume(current)");
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
            ok &= expect_ok(hook.sendHeartbeat(), "hook sendHeartbeat()");
            uint16_t heartbeat_value = 0;
            ok &= expect_ok(hook.readHeartbeat(heartbeat_value), "hook readHeartbeat()");
            if (ok) {
                std::cout << "[INFO] hook heartbeat=" << heartbeat_value << '\n';
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
            std::cout << "[INFO] hook valid_index=0x" << std::hex << valid_index << std::dec
                      << " valid_entries=" << entries.size() << '\n';
            for (int i = 0; i < HookWarning::NUM_RFID_INDICES; ++i) {
                const bool valid = hook.isRfidValid(i);
                const RfidEntry entry = hook.getRfidEntry(i);
                const uint32_t uid = hook.getRfidUid(i);
                const uint8_t signal = hook.getRfidSignalStrength(i);
                const uint8_t battery = hook.getRfidBattery(i);
                if (valid) {
                    std::cout << "[INFO] RFID[" << i << "] uid=" << uid
                              << " signal=" << static_cast<int>(signal)
                              << " battery=" << static_cast<int>(battery)
                              << " entry_uid=" << entry.uid << '\n';
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
            ok &= expect_ok(hook.slot_warning(signal_index), "hook " + name);
            ok &= expect_ok(hook.slot_warning(0), "hook slot_warning(0) restore");
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
            const bool ok = expect_ok(hook.run(), "hook run()");
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
    if (!expect_ok(trolley.readStatus(status), "trolley readStatus()")) {
        return false;
    }
    std::cout << std::fixed << std::setprecision(2)
              << "[INFO] trolley status dump\n"
              << "  power_3v3_on=" << static_cast<int>(status.power_3v3_on) << '\n'
              << "  power_5v_on=" << static_cast<int>(status.power_5v_on) << '\n'
              << "  power_cctv_on=" << static_cast<int>(status.power_cctv_on) << '\n'
              << "  power_4g_on=" << static_cast<int>(status.power_4g_on) << '\n'
              << "  standby_mode_active=" << static_cast<int>(status.standby_mode_active) << '\n'
              << "  bms_charging=" << static_cast<int>(status.bms_charging) << '\n'
              << "  bridge_ping_ok=" << static_cast<int>(status.bridge_ping_ok) << '\n'
              << "  battery_bms_read_ok=" << static_cast<int>(status.battery_bms_read_ok) << '\n'
              << "  mppt_read_ok=" << static_cast<int>(status.mppt_read_ok) << '\n'
              << "  laser_1_read_ok=" << static_cast<int>(status.laser_1_read_ok) << '\n'
              << "  laser_2_read_ok=" << static_cast<int>(status.laser_2_read_ok) << '\n'
              << "  cctv_ping_ok=" << static_cast<int>(status.cctv_ping_ok) << '\n'
              << "  system_version=" << status.system_version << '\n'
              << "  system_error_code=" << status.system_error_code << '\n'
              << "  battery_version=" << status.battery_version << '\n'
              << "  battery_level=" << static_cast<int>(status.battery_level) << '\n'
              << "  discharge_time_valid=" << static_cast<int>(status.discharge_time_valid) << '\n'
              << "  discharge_time_min=" << status.discharge_time_min << '\n'
              << "  charge_time_valid=" << static_cast<int>(status.charge_time_valid) << '\n'
              << "  charge_time_min=" << status.charge_time_min << '\n'
              << "  battery_voltage_v=" << status.battery_voltage_v << '\n'
              << "  battery_current_a=" << status.battery_current_a << '\n'
              << "  laser_distance[0]=" << status.laser_distance[0] << '\n'
              << "  laser_distance[1]=" << status.laser_distance[1] << '\n'
              << "  laser_distance[2]=" << status.laser_distance[2] << '\n'
              << "  mppt_charge_status=" << status.mppt_charge_status << '\n'
              << "  shutdown_time=" << status.shutdown_time << '\n'
              << "  startup_time=" << status.startup_time << '\n'
              << "  work_mode=" << status.work_mode << '\n'
              << "  rtc_sys_time_diff=" << status.rtc_sys_time_diff << '\n'
              << "  rtc_hour=" << status.rtc_hour << '\n'
              << "  rtc_minute=" << status.rtc_minute << '\n'
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
            if (!prompt_int_in_range("输入 3V3 电源目标值", status.power_3v3_on, 0, 1, target)) return true;
            return expect_ok(trolley.setPower3v3(static_cast<std::uint8_t>(target)), "trolley setPower3v3(target)");
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
            if (!prompt_int_in_range("输入 5V 电源目标值", status.power_5v_on, 0, 1, target)) return true;
            return expect_ok(trolley.setPower5v(static_cast<std::uint8_t>(target)), "trolley setPower5v(target)");
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
            if (!prompt_int_in_range("输入 CCTV 电源目标值", status.power_cctv_on, 0, 1, target)) return true;
            return expect_ok(trolley.setPowerCctv(static_cast<std::uint8_t>(target)), "trolley setPowerCctv(target)");
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
            if (!prompt_int_in_range("输入 4G 电源目标值", status.power_4g_on, 0, 1, target)) return true;
            return expect_ok(trolley.setPower4g(static_cast<std::uint8_t>(target)), "trolley setPower4g(target)");
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
                             "trolley setStandbyEnable(target)");
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
                             "trolley setStandbyPowerMode(target)");
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
            if (!prompt_int_in_range("输入工作模式目标值", status.work_mode, 0, 65535, target)) return true;
            return expect_ok(trolley.setWorkMode(static_cast<uint16_t>(target)), "trolley setWorkMode(target)");
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
            if (!prompt_int_in_range("输入 RTC 小时", status.rtc_hour, 0, 23, hour)) return true;
            if (!prompt_int_in_range("输入 RTC 分钟", status.rtc_minute, 0, 59, minute)) return true;
            return expect_ok(trolley.setRtcTime(static_cast<uint16_t>(hour), static_cast<uint16_t>(minute)),
                             "trolley setRtcTime(target)");
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
            if (!prompt_int_in_range("输入开机时间目标值", status.startup_time, 0, 65535, target)) return true;
            return expect_ok(trolley.setStartupTime(static_cast<uint16_t>(target)), "trolley setStartupTime(target)");
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
            if (!prompt_int_in_range("输入关机时间目标值", status.shutdown_time, 0, 65535, target)) return true;
            return expect_ok(trolley.setShutdownTime(static_cast<uint16_t>(target)), "trolley setShutdownTime(target)");
        });
}

bool trolley_test_reboot(const ModbusConfig& config) {
    if (!prompt_yes_no("确认触发 trolley reboot? 这是高风险操作")) {
        std::cout << "[INFO] 已取消 trolley reboot 测试\n";
        return true;
    }
    return with_connected_device<TrolleyControl>(
        "trolley",
        [&]() { return TrolleyControl(config.trolley_ip, config.trolley_port, config.trolley_slave); },
        [&](TrolleyControl& trolley) { return expect_ok(trolley.triggerReboot(), "trolley triggerReboot()"); });
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
    return expect_ok(encoder.readEncoderPosition(position), "encoder readEncoderPosition()");
}

bool encoder_read_settings(MultiTurnEncoderRTU& encoder, MultiTurnEncoderRTU::EncoderSettings& settings) {
    return expect_ok(encoder.readEncoderSettings(settings), "encoder readEncoderSettings()");
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
                std::cout << "[INFO] encoder position=" << position
                          << " addr=" << settings.deviceAddress
                          << " baud=" << static_cast<int>(settings.baudRate)
                          << " direction=" << static_cast<int>(settings.countingDirection)
                          << " parity=" << static_cast<int>(settings.parityCheck)
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
                                      "encoder readEncoderNumberOfTurns()");
            if (ok) {
                std::cout << "[INFO] encoder turns=" << turns
                          << " timestamp=" << timestamp
                          << " duration=" << duration << '\n';
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
            return expect_ok(encoder.writeEncoderPosition(position), "encoder writeEncoderPosition(current)");
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
            const bool ok = expect_ok(encoder.readRawSettings(regs), "encoder readRawSettings()");
            if (ok) {
                std::cout << "[INFO] raw settings: [" << regs[0] << ", " << regs[1] << ", " << regs[2]
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
            if (!expect_ok(ok, "encoder getEncoderSettings()")) {
                return false;
            }
            std::cout << "[INFO] encoder settings: addr=" << settings.deviceAddress
                      << " baud=" << settings.baudRate
                      << " direction=" << settings.countingDirection
                      << " parity=" << settings.parityCheck << '\n';
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
                             "encoder write485DeviceAddress(current)");
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
            return expect_ok(encoder.writeBaudRate(settings.baudRate), "encoder writeBaudRate(current)");
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
                             "encoder writeCountingDirection(current)");
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
                             "encoder writeParityCheck(current)");
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
            std::cout << "[INFO] encoder getRunLoopSleepMs=" << encoder.getRunLoopSleepMs() << '\n'
                      << "[INFO] encoder data timestamp=" << data.timestamp
                      << " variance=" << data.time_variance
                      << " value=" << data.value << '\n'
                      << "[INFO] encoder getDataTimestamp=" << timestamp << '\n'
                      << "[INFO] encoder_data timestamp=" << encoder_data.timestamp
                      << " variance=" << encoder_data.time_variance
                      << " value=" << encoder_data.value
                      << " velocity=" << encoder_data.velocity << '\n';
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
                           "encoder readEncoderNumberOfTurns() before updateEncoderData")) {
                return false;
            }
            for (int i = 0; i < 120; ++i) {
                encoder.updateEncoderData(turns + static_cast<double>(i) * 0.001,
                                          timestamp + static_cast<double>(i) * 0.01,
                                          duration);
            }
            const double velocity = encoder.computeVelocity();
            const auto encoder_data = encoder.getEncoderData();
            std::cout << "[INFO] encoder computed velocity=" << velocity
                      << " latest value=" << encoder_data.value
                      << " latest velocity=" << encoder_data.velocity << '\n';
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
            return expect_ok(true, "encoder run()/stop()");
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
        std::cout << (ok ? "[INFO] Hook 菜单测试完成\n" : "[INFO] Hook 菜单测试失败\n");
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
        std::cout << (ok ? "[INFO] Trolley 菜单测试完成\n" : "[INFO] Trolley 菜单测试失败\n");
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
        std::cout << (ok ? "[INFO] Encoder 菜单测试完成\n" : "[INFO] Encoder 菜单测试失败\n");
    }
}

void run_all_devices_menu(const ModbusConfig& config) {
    bool ok = true;
    ok &= hook_test_all(config);
    ok &= trolley_test_all(config);
    ok &= encoder_test_all(config);
    std::cout << (ok ? "[INFO] 全部设备测试完成\n" : "[INFO] 全部设备测试存在失败\n");
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
                 "HookWarning 功能测试",
                 "TrolleyControl 功能测试",
                 "MultiTurnEncoderRTU 功能测试",
                 "全部设备顺序测试"});

            switch (choice) {
                case 0:
                    std::cout << "[INFO] 退出测试程序\n";
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
                            std::cout << "[INFO] 配置文件切换成功\n";
                        } else {
                            std::cout << "[WARN] 配置文件切换失败，保留原配置\n";
                        }
                    }
                    break;
                }
                case 3:
                    run_hook_menu(config);
                    break;
                case 4:
                    run_trolley_menu(config);
                    break;
                case 5:
                    run_encoder_menu(config);
                    break;
                case 6:
                    run_all_devices_menu(config);
                    break;
                default:
                    break;
            }
        }
    } catch (const std::exception& ex) {
        std::cerr << "[FAIL] unhandled exception: " << ex.what() << '\n';
    } catch (...) {
        std::cerr << "[FAIL] unknown exception\n";
    }
    return EXIT_FAILURE;
}
