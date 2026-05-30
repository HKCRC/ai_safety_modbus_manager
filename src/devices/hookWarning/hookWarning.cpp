#include "devices/hookWarning/hookWarning.h"
#include "common/modbus_control.h"
#include <memory>
#include <iostream>
#include <vector>

HookWarning::HookWarning(const char* device, int baud, char parity, int data_bit, int stop_bit, int slave_id)
    : ModbusControl(device, baud, parity, data_bit, stop_bit, slave_id) {}

HookWarning::HookWarning(const char* device, int baud, char parity, int data_bit, int stop_bit, int slave_id, uint32_t to_sec, uint32_t to_usec)
    : ModbusControl(device, baud, parity, data_bit, stop_bit, slave_id, to_sec, to_usec) {}

HookWarning::HookWarning(const std::string& device, int baud, char parity, int data_bit, int stop_bit, int slave_id)
    : HookWarning(device.c_str(), baud, parity, data_bit, stop_bit, slave_id) {}

HookWarning::HookWarning(const std::string& device, int baud, char parity, int data_bit, int stop_bit, int slave_id, uint32_t to_sec, uint32_t to_usec)
    : HookWarning(device.c_str(), baud, parity, data_bit, stop_bit, slave_id, to_sec, to_usec) {}

HookWarning::~HookWarning() {
    disconnect();
    if (running_thread_.joinable()) {
        running_thread_.join();
    }
}

bool HookWarning::connect() {
    if (!ModbusControl::connect()) {
        return false;
    }
    return initializeDevice();
}

void HookWarning::disconnect() {
    ModbusControl::disconnect();
    running = false;
}

bool HookWarning::isConnected() const {
    return ModbusControl::isConnected();
}

bool HookWarning::initializeDevice() {
    std::cout << "Initializing HookWarning device (simplified)..." << std::endl;
    std::cout << "Device initialized successfully (basic connection only)" << std::endl;
    return true;
}

bool HookWarning::setStrobe(bool on) {
    strobe_on = on;
    return writeRegister(REG_CTRL_STROBE, on ? 1u : 0u);
}

bool HookWarning::setSpeaker7m(bool on) {
    speaker_7m_on = on;
    return writeRegister(REG_CTRL_SPEAKER_7M, on ? 1u : 0u);
}

bool HookWarning::setSpeaker3m(bool on) {
    speaker_3m_on = on;
    return writeRegister(REG_CTRL_SPEAKER_3M, on ? 1u : 0u);
}

bool HookWarning::setSpeakerOff() {
    return writeRegister(REG_CTRL_SPEAKER_7M, 0u) && writeRegister(REG_CTRL_SPEAKER_3M, 0u);
}

bool HookWarning::readStrobeStatus(bool& on) {
    uint16_t val = 0;
    if (!readHoldingRegisters(REG_STATUS_STROBE, 1, &val)) {
        return false;
    }
    on = (val != 0);
    return true;
}

bool HookWarning::readSpeakerStatus(bool& on) {
    uint16_t val = 0;
    if (!readHoldingRegisters(REG_STATUS_SPEAKER, 1, &val)) {
        return false;
    }
    on = (val != 0);
    return true;
}

bool HookWarning::readBatteryLevel(uint16_t& level) {
    uint16_t val = 0;
    if (!readHoldingRegisters(REG_BATTERY, 1, &val)) {
        return false;
    }
    level = val;
    return true;
}

bool HookWarning::readStatus() {
    // 先尝试读取基本状态寄存器
    uint16_t status_regs[4] = {0};
    if (!readHoldingRegisters(REG_STATUS_STROBE, 4, status_regs)) {
        std::cerr << "Failed to read basic status registers" << std::endl;
        return false;
    }
    
    // 更新基本状态
    strobe_on_feedback = (status_regs[0] != 0);
    speaker_on_feedback = (status_regs[1] != 0);
    
    // battery_level from modbus is typically 10000=100%, convert to 0-100%
    const int p = static_cast<int>(status_regs[2]) / 100;
    battery_level_feedback = static_cast<uint16_t>(std::max(0, std::min(p, 100)));
    
    volume_feedback = status_regs[3];
    
    // 读取其他状态寄存器
    uint16_t extended_regs[12] = {0};
    if (readHoldingRegisters(REG_DISCHARGE_TIME, 12, extended_regs)) {
        discharge_time_feedback = extended_regs[0];
        workmode_feedback = extended_regs[1];
        charging_status_feedback = extended_regs[2];
        charge_time_feedback = extended_regs[3];
        voltage_feedback = extended_regs[4] * 0.01f;
        current_feedback = extended_regs[5] * 0.01f;
        shutdown_time_feedback = extended_regs[6];
        startup_time_feedback = extended_regs[7];
        error_code_feedback = extended_regs[8];
        stm_rtc_time_feedback = extended_regs[9];
        standby_enable_feedback = extended_regs[10];
        pc_rtc_time_feedback = extended_regs[11];
    } else {
        uint16_t fallback_regs[9] = {0};
        if (!readHoldingRegisters(REG_DISCHARGE_TIME, 9, fallback_regs)) {
            return false;
        }
        discharge_time_feedback = fallback_regs[0];
        workmode_feedback = fallback_regs[1];
        charging_status_feedback = fallback_regs[2];
        charge_time_feedback = fallback_regs[3];
        voltage_feedback = fallback_regs[4] * 0.01f;
        current_feedback = fallback_regs[5] * 0.01f;
        shutdown_time_feedback = fallback_regs[6];
        startup_time_feedback = fallback_regs[7];
        error_code_feedback = fallback_regs[8];
        stm_rtc_time_feedback = 0;
        standby_enable_feedback = 0;
        pc_rtc_time_feedback = 0;
    }
    
    timestamp = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    
    // 只在调试模式下打印状态信息
    // std::cout << "Battery level: " << battery_level_feedback << std::endl;
    // std::cout << "Voltage: " << voltage_feedback << "V, Current: " << current_feedback << "A" << std::endl;
    // std::cout << "Work mode: " << workmode_feedback << ", Charging: " << charging_status_feedback << std::endl;
    
    return true;
}

bool HookWarning::refreshStatus() {
    return readStatus();
}

bool HookWarning::slot_warning(int signal_index) {
    switch (signal_index) {
        case -1:
            return setSpeaker3m(false) && setStrobe((speaker_7m_on ? true : false));
        case -2:
            return setSpeaker7m(false) && setStrobe((speaker_3m_on ? true : false));
        case 0:
            return setSpeakerOff() && setStrobe(false);
        case 1:
            return setSpeaker3m(true) && setStrobe(true);
        case 2:
            return setSpeaker7m(true) && setStrobe(true);
        default:
            return false;
    }
}

bool HookWarning::run() {
    if (running) {
        std::cout << "Thread already running" << std::endl;
        return true;
    }
    
    running = true;
    running_thread_ = std::thread([this]() {
        while (running) {
            readStatus();
            sendHeartbeat();
            std::this_thread::sleep_for(std::chrono::milliseconds(5000));
        }
    });
    return true;
}

uint16_t HookWarning::get_battery_level_feedback() {
    double current_timestamp = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    if (current_timestamp - timestamp > 10) {
        readStatus();
        timestamp = current_timestamp;
    }
    return (current_timestamp - timestamp < timeout) ? battery_level_feedback : 0;
}

bool HookWarning::setVolume(uint16_t level) {
    if (level > 30) level = 30;
    return writeRegister(REG_VOLUME, level);
}

bool HookWarning::sendHeartbeat() {
    return writeRegister(REG_HEARTBEAT, heartbeat_counter++);
}

bool HookWarning::readHeartbeat(uint16_t& value) {
    return readHoldingRegisters(REG_HEARTBEAT, 1, &value);
}

uint16_t HookWarning::get_volume_feedback() {
    if (readHoldingRegisters(REG_VOLUME, 1, &volume_feedback))
        return volume_feedback;
    return 0;
}

uint16_t HookWarning::getValidIndex() const {
    std::lock_guard<std::mutex> lock(rfid_mutex_);
    return valid_index_;
}

bool HookWarning::isRfidValid(int index) const {
    if (index < 0 || index >= NUM_RFID_INDICES) return false;
    std::lock_guard<std::mutex> lock(rfid_mutex_);
    return (valid_index_ & (1u << index)) != 0;
}

RfidEntry HookWarning::getRfidEntry(int index) const {
    RfidEntry e = {};
    if (index < 0 || index >= NUM_RFID_INDICES) return e;
    std::lock_guard<std::mutex> lock(rfid_mutex_);
    e = rfid_entries_[index];
    return e;
}

uint32_t HookWarning::getRfidUid(int index) const {
    if (index < 0 || index >= NUM_RFID_INDICES) return 0;
    std::lock_guard<std::mutex> lock(rfid_mutex_);
    return rfid_entries_[index].uid;
}

uint8_t HookWarning::getRfidSignalStrength(int index) const {
    if (index < 0 || index >= NUM_RFID_INDICES) return 0;
    std::lock_guard<std::mutex> lock(rfid_mutex_);
    return rfid_entries_[index].signal_strength;
}

uint8_t HookWarning::getRfidBattery(int index) const {
    if (index < 0 || index >= NUM_RFID_INDICES) return 0;
    std::lock_guard<std::mutex> lock(rfid_mutex_);
    return rfid_entries_[index].battery;
}

std::vector<std::pair<int, RfidEntry>> HookWarning::getValidRfidEntries() const {
    std::vector<std::pair<int, RfidEntry>> result;
    std::lock_guard<std::mutex> lock(rfid_mutex_);
    for (int i = 0; i < NUM_RFID_INDICES; ++i) {
        if ((valid_index_ & (1u << i)) != 0)
            result.emplace_back(i, rfid_entries_[i]);
    }
    return result;
}

// Additional status getters implementation
uint16_t HookWarning::getDischargeTime() const {
    return discharge_time_feedback;
}

uint16_t HookWarning::getWorkmode() const {
    return workmode_feedback;
}

uint16_t HookWarning::getChargingStatus() const {
    return charging_status_feedback;
}

uint16_t HookWarning::getChargeTime() const {
    return charge_time_feedback;
}

float HookWarning::getVoltage() const {
    return voltage_feedback;
}

float HookWarning::getCurrent() const {
    return current_feedback;
}

uint16_t HookWarning::getShutdownTime() const {
    return shutdown_time_feedback;
}

uint16_t HookWarning::getStartupTime() const {
    return startup_time_feedback;
}

uint16_t HookWarning::getErrorCode() const {
    return error_code_feedback;
}

uint16_t HookWarning::getStmRtcTime() const {
    return stm_rtc_time_feedback;
}

uint16_t HookWarning::getStandbyEnable() const {
    return standby_enable_feedback;
}

uint16_t HookWarning::getPcRtcTime() const {
    return pc_rtc_time_feedback;
}
