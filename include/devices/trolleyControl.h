#ifndef TROLLEY_CONTROL_H
#define TROLLEY_CONTROL_H

#include <array>
#include <cstdint>
#include <string>

#include "common/modbus_control.h"

struct TrolleyStatus {
    // All status flags are normalized for upper layers:
    // 1 = on/active/charging/ok/success, 0 = off/inactive/not charging/fail.
    std::uint8_t power_3v3_on = 0;          // 1=3.3V power on, 0=3.3V power off
    std::uint8_t power_5v_on = 0;           // 1=5V power on, 0=5V power off
    std::uint8_t power_cctv_on = 0;         // 1=CCTV power on, 0=CCTV power off
    std::uint8_t power_4g_on = 0;           // 1=4G power on, 0=4G power off
    std::uint8_t standby_mode_active = 0;   // 1=standby active, 0=standby inactive
    std::uint8_t bms_charging = 0;          // 1=battery charging, 0=not charging
    std::uint8_t bridge_ping_ok = 1;        // 1=bridge reachable, 0=bridge ping failed
    std::uint8_t battery_bms_read_ok = 1;   // 1=battery BMS read success, 0=read failed
    std::uint8_t mppt_read_ok = 1;          // 1=MPPT read success, 0=read failed
    std::uint8_t laser_1_read_ok = 1;       // 1=laser 1 read success, 0=read failed
    std::uint8_t laser_2_read_ok = 1;       // 1=laser 2 read success, 0=read failed
    std::uint8_t cctv_ping_ok = 1;          // 1=CCTV reachable, 0=CCTV ping failed

    uint32_t system_version = 0;            // raw system version value from input registers [0..1]
    uint32_t system_error_code = 0;         // raw system error code from input registers [2..3]

    uint16_t battery_version = 0;           // raw battery/BMS version value
    std::uint8_t battery_level = 0;         // battery level percentage, 0-100
    std::uint8_t discharge_time_valid = 0;  // 1=discharge_time_min is valid, 0=invalid/unavailable
    uint16_t discharge_time_min = 0;        // remaining discharge time in minutes
    std::uint8_t charge_time_valid = 0;     // 1=charge_time_min is valid, 0=invalid/unavailable
    uint16_t charge_time_min = 0;           // remaining charge time in minutes
    float battery_voltage_v = 0.0f;         // battery voltage, unit V
    float battery_current_a = 0.0f;         // battery current, unit A
    std::array<uint16_t, 3> laser_distance = {0, 0, 0}; // raw laser distance registers [32..34]
    uint16_t mppt_charge_status = 0;        // MPPT status raw value, 0=not charging, 1=charging, 2=error, 3=not applicable/read failed

    uint16_t shutdown_time = 0;             // raw shutdown time register value
    uint16_t startup_time = 0;              // raw startup time register value
    uint16_t work_mode = 0;                 // raw work mode register value
    uint16_t rtc_sys_time_diff = 0;         // raw RTC-system time difference register value
    uint16_t rtc_hour = 0;                  // RTC hour, range 0..23
    uint16_t rtc_minute = 0;                // RTC minute, range 0..59
    uint16_t device_status = 0;             // 0=standby, 1=active, 2=error
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

    static constexpr int COIL_STATUS_3V3 = 16;
    static constexpr int COIL_STATUS_5V = 17;
    static constexpr int COIL_STATUS_CCTV = 18;
    static constexpr int COIL_STATUS_4G = 19;
    static constexpr int COIL_STATUS_STANDBY_ACTIVE = 20;
    static constexpr int COIL_STATUS_BMS_CHARGING = 21;
    static constexpr int COIL_STATUS_BRIDGE_PING_FAIL = 22;
    static constexpr int COIL_STATUS_BMS_READ_FAIL = 23;
    static constexpr int COIL_STATUS_MPPT_READ_FAIL = 24;
    static constexpr int COIL_STATUS_LASER_1_READ_FAIL = 25;
    static constexpr int COIL_STATUS_LASER_2_READ_FAIL = 26;
    static constexpr int COIL_STATUS_CCTV_PING_FAIL = 27;

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
    static constexpr int IREG_LASER_DISTANCE_3 = 34;

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
    bool setWorkMode(uint16_t mode);
    bool setRtcTime(uint16_t hour, uint16_t minute);
    bool setStartupTime(uint16_t value);
    bool setShutdownTime(uint16_t value);

    bool readStatus(TrolleyStatus& out);

private:
    bool writeOnOff(int addr, std::uint8_t value);
};

#endif // TROLLEY_CONTROL_H
