#include "devices/trolleyControl.h"

namespace {

std::uint8_t to_flag(bool value) {
    return value ? 1u : 0u;
}

std::uint8_t percent_from_10000(std::uint16_t v) {
    const int p = static_cast<int>(v) / 100;
    const int bounded = std::max(0, std::min(p, 100));
    return static_cast<std::uint8_t>(bounded);
}

} // namespace

TrolleyControl::TrolleyControl(const std::string& ip_address, int port, int slave_address)
    : ModbusControl(ip_address.c_str(), port, slave_address) {}

TrolleyControl::TrolleyControl(const char* ip_address, int port, int slave_address)
    : ModbusControl(ip_address, port, slave_address) {}

bool TrolleyControl::writeOnOff(int addr, std::uint8_t value) {
    const bool on = (value != 0u);
    const bool is_inverted_power_control =
        (addr == COIL_CTRL_3V3) || (addr == COIL_CTRL_5V) || (addr == COIL_CTRL_CCTV) || (addr == COIL_CTRL_4G);
    const std::uint16_t modbus_value = is_inverted_power_control ? (on ? 0u : 1u) : (on ? 1u : 0u);
    if (writeCoil(addr, modbus_value)) {
        return true;
    }
    return writeRegister(addr, modbus_value);
}

bool TrolleyControl::triggerReboot() {
    return writeOnOff(COIL_REBOOT, true);
}

bool TrolleyControl::setPower3v3(std::uint8_t value) {
    return writeOnOff(COIL_CTRL_3V3, value);
}

bool TrolleyControl::setPower5v(std::uint8_t value) {
    return writeOnOff(COIL_CTRL_5V, value);
}

bool TrolleyControl::setPowerCctv(std::uint8_t value) {
    return writeOnOff(COIL_CTRL_CCTV, value);
}

bool TrolleyControl::setPower4g(std::uint8_t value) {
    return writeOnOff(COIL_CTRL_4G, value);
}

bool TrolleyControl::setStandbyEnable(std::uint8_t value) {
    return writeOnOff(COIL_CTRL_STANDBY_ENABLE, value);
}

bool TrolleyControl::setStandbyPowerMode(std::uint8_t value) {
    return writeOnOff(COIL_CTRL_STANDBY_POWER_MODE, value);
}

bool TrolleyControl::setWorkMode(uint16_t mode) {
    return writeRegister(HREG_WORK_MODE, mode);
}

bool TrolleyControl::setRtcTime(uint16_t hour, uint16_t minute) {
    return writeRegister(HREG_RTC_HOUR, hour) && writeRegister(HREG_RTC_MINUTE, minute);
}

bool TrolleyControl::setStartupTime(uint16_t value) {
    return writeRegister(HREG_STARTUP_TIME, value);
}

bool TrolleyControl::setShutdownTime(uint16_t value) {
    return writeRegister(HREG_SHUTDOWN_TIME, value);
}

bool TrolleyControl::readStatus(TrolleyStatus& out) {
    uint8_t bits[12] = {0};
    if (!readCoils(COIL_STATUS_3V3, 12, bits)) {
        return false;
    }

    out.power_3v3_on = to_flag(bits[0] == 0);
    out.power_5v_on = to_flag(bits[1] == 0);
    out.power_cctv_on = to_flag(bits[2] == 0);
    out.power_4g_on = to_flag(bits[3] == 0);
    out.standby_mode_active = to_flag(bits[4] != 0);
    out.bms_charging = to_flag(bits[5] != 0);
    out.bridge_ping_ok = to_flag(bits[6] == 0);
    out.battery_bms_read_ok = to_flag(bits[7] == 0);
    out.mppt_read_ok = to_flag(bits[8] == 0);
    out.laser_1_read_ok = to_flag(bits[9] == 0);
    out.laser_2_read_ok = to_flag(bits[10] == 0);
    out.cctv_ping_ok = to_flag(bits[11] == 0);

    uint16_t system_regs[4] = {0};
    if (readInputRegisters(IREG_SYSTEM_VERSION_HIGH, 4, system_regs)) {
        out.system_version =
            (static_cast<uint32_t>(system_regs[0]) << 16) | static_cast<uint32_t>(system_regs[1]);
        out.system_error_code =
            (static_cast<uint32_t>(system_regs[2]) << 16) | static_cast<uint32_t>(system_regs[3]);
    } else {
        out.system_version = 0;
        out.system_error_code = 0;
    }

    uint16_t holding[7] = {0};
    if (!readHoldingRegisters(HREG_WORK_MODE, 7, holding)) {
        return false;
    }
    out.work_mode = holding[0];
    out.rtc_sys_time_diff = holding[1];
    out.rtc_hour = holding[2];
    out.rtc_minute = holding[3];
    out.startup_time = holding[4];
    out.shutdown_time = holding[5];
    out.device_status = holding[6];

    uint16_t batt_regs[6] = {0};
    if (readInputRegisters(IREG_BATTERY_VERSION, 6, batt_regs)) {
        out.battery_version = batt_regs[0];
        out.battery_level = percent_from_10000(batt_regs[1]);
        out.discharge_time_valid = to_flag(batt_regs[2] != 0xFFFFu);
        out.discharge_time_min = out.discharge_time_valid ? batt_regs[2] : 0;
        out.charge_time_valid = to_flag(batt_regs[3] != 0xFFFFu);
        out.charge_time_min = out.charge_time_valid ? batt_regs[3] : 0;
        out.battery_voltage_v = static_cast<float>(batt_regs[4]) * 0.01f;
        out.battery_current_a = static_cast<float>(batt_regs[5]) * 0.01f;
    } else {
        out.battery_version = 0;
        out.battery_level = 0;
        out.discharge_time_valid = 0;
        out.discharge_time_min = 0;
        out.charge_time_valid = 0;
        out.charge_time_min = 0;
        out.battery_voltage_v = 0.0f;
        out.battery_current_a = 0.0f;
    }

    uint16_t mppt_status = 0;
    if (readInputRegisters(IREG_MPPT_CHARGE_STATUS, 1, &mppt_status)) {
        out.mppt_charge_status = mppt_status;
    } else {
        out.mppt_charge_status = 3;
        out.mppt_read_ok = 0;
    }

    uint16_t laser_regs[3] = {0};
    if (readInputRegisters(IREG_LASER_DISTANCE_1, 3, laser_regs)) {
        out.laser_distance = {laser_regs[0], laser_regs[1], laser_regs[2]};
    } else {
        out.laser_distance = {0, 0, 0};
        out.laser_1_read_ok = 0;
        out.laser_2_read_ok = 0;
    }

    return true;
}
