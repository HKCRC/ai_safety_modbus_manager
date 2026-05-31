#ifndef TROLLEY_CONTROL_H
#define TROLLEY_CONTROL_H

#include <array>
#include <cstdint>
#include <string>

#include "common/modbus_control.h"

struct TrolleyStatus {
    // --- 1. 小车使用状态相关 ---
    std::uint8_t power_saving_mode_active = 0; // 1=power saving active, 0=inactive
    std::uint8_t sleep_mode_active = 0;     // 1=sleep mode active, 0=normal mode
    uint16_t device_status = 0;             // 0=standby, 1=active, 2=error
    
    // b. 如果异常，读异常信息：
    uint32_t system_error_code = 0;         // raw system error code from input registers [2..3]
    
    // c. 离线原因判断辅助状态
    std::uint8_t bridge_ping_ok = 1;        // 1=bridge reachable, 0=bridge ping failed
    std::uint8_t bms_read_ok = 1;           // 1=ok, 0=read fail
    std::uint8_t mppt_read_ok = 1;          // 1=ok, 0=read fail
    std::uint8_t laser_1_read_ok = 1;       // 1=ok, 0=read fail
    std::uint8_t laser_2_read_ok = 1;       // 1=ok, 0=read fail
    std::uint8_t cctv_ping_ok = 1;          // 1=ok, 0=ping fail
    
    // --- 2. 小车电池信息 ---
    std::uint8_t battery_level = 0;         // a. 电量百分比 %
    float battery_voltage_v = 0.0f;         // b. 电压 V
    float battery_current_a = 0.0f;         // c. 电流 A
    std::uint8_t bms_charging = 0;          // d. 是否在充电 0/1
    uint16_t charge_time_min = 0;           // e. 剩余充电时间 min
    uint16_t discharge_time_min = 0;        // f. 剩余使用时间 min
    std::uint8_t discharge_time_valid = 0;  
    std::uint8_t charge_time_valid = 0;     

    // --- 3. MPPT信息 ---
    uint16_t mppt_charge_status = 0;        // a. 充电状态 (充电中/未充电/错误/不适用)

    // --- 4. 激光测距 ---
    std::array<uint16_t, 2> laser_distance = {0, 0}; // a/b. 激光测距距离
};

class TrolleyControl : public ModbusControl {
public:
    static constexpr int COIL_REBOOT = 0;
    static constexpr int COIL_CTRL_3V3 = 1;
    static constexpr int COIL_CTRL_5V = 2;
    static constexpr int COIL_CTRL_CCTV = 3;
    static constexpr int COIL_CTRL_4G = 4;
    static constexpr int COIL_CTRL_STANDBY_ENABLE = 5;
    static constexpr int COIL_CTRL_STANDBY_POWER_MODE = 6;
    static constexpr int COIL_CTRL_SLEEP_MODE = 7;

    static constexpr int COIL_STATUS_3V3 = 16;
    static constexpr int COIL_STATUS_5V = 17;
    static constexpr int COIL_STATUS_CCTV = 18;
    static constexpr int COIL_STATUS_4G = 19;
    static constexpr int COIL_STATUS_POWER_SAVING_MODE = 20;
    static constexpr int COIL_STATUS_BMS_CHARGING = 21;
    static constexpr int COIL_STATUS_BRIDGE_PING_FAIL = 22;
    static constexpr int COIL_STATUS_BMS_READ_FAIL = 23;
    static constexpr int COIL_STATUS_MPPT_READ_FAIL = 24;
    static constexpr int COIL_STATUS_LASER_1_READ_FAIL = 25;
    static constexpr int COIL_STATUS_LASER_2_READ_FAIL = 26;
    static constexpr int COIL_STATUS_CCTV_PING_FAIL = 27;
    static constexpr int COIL_STATUS_SLEEP_MODE = 28;

    static constexpr int IREG_SYSTEM_VERSION_HIGH = 0;
    static constexpr int IREG_SYSTEM_VERSION_LOW = 1;
    static constexpr int IREG_SYSTEM_ERROR_HIGH = 2;
    static constexpr int IREG_SYSTEM_ERROR_LOW = 3;
    static constexpr int IREG_BATTERY_VERSION = 16;
    static constexpr int IREG_BATTERY_LEVEL = 17;
    static constexpr int IREG_DISCHARGE_TIME = 18;
    static constexpr int IREG_CHARGE_TIME = 19;
    static constexpr int IREG_BATTERY_VOLTAGE = 20;
    static constexpr int IREG_BATTERY_CURRENT = 21;
    static constexpr int IREG_MPPT_CHARGE_STATUS = 25;
    static constexpr int IREG_LASER_DISTANCE_1 = 32;
    static constexpr int IREG_LASER_DISTANCE_2 = 33;

    static constexpr int HREG_WORK_MODE = 0;
    static constexpr int HREG_RTC_SYS_TIME_DIFF = 1;
    static constexpr int HREG_RTC_HOUR = 2;
    static constexpr int HREG_RTC_MINUTE = 3;
    static constexpr int HREG_STARTUP_TIME = 4;
    static constexpr int HREG_SHUTDOWN_TIME = 5;
    static constexpr int HREG_DEVICE_STATUS = 6;

    explicit TrolleyControl(const std::string& ip_address, int port = 502, int slave_address = 1);
    TrolleyControl(const char* ip_address, int port, int slave_address);

    bool triggerReboot();
    bool setPower3v3(std::uint8_t value);          // 1=on, 0=off
    bool setPower5v(std::uint8_t value);           // 1=on, 0=off
    bool setPowerCctv(std::uint8_t value);         // 1=on, 0=off
    bool setPower4g(std::uint8_t value);           // 1=on, 0=off
    bool setStandbyEnable(std::uint8_t value);     // 1=enable, 0=disable
    bool setStandbyPowerMode(std::uint8_t value);  // 1=enable, 0=disable
    bool setSleepMode(std::uint8_t value);         // 1=sleep, 0=normal
    bool setWorkMode(uint16_t mode);
    bool setRtcTime(uint16_t hour, uint16_t minute);
    bool setStartupTime(uint16_t value);
    bool setShutdownTime(uint16_t value);

    bool readStatus(TrolleyStatus& out);

private:
    bool writeOnOff(int addr, std::uint8_t value);
};

#endif // TROLLEY_CONTROL_H
