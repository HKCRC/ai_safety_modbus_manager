#ifndef HOOK_WARNING_H
#define HOOK_WARNING_H

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "common/modbus_control.h"

/**
 * HookWarning - Modbus RTU device for strobe light and speaker (7m/3m) control,
 * with status and battery level readback.
 *
 * Holding register map:
 *   0     Control strobe: 1=on, 0=off
 *   1     Control speaker 7m: 1=sound, 0=stop
 *   2     Control speaker 3m: when 7m is on, 1=3m sound / 0=7m sound; stop = both 1 and 2 are 0
 *   100   Strobe status: 1=on, 0=off (read-only)
 *   101   Speaker status: 1=on, 0=off (read-only)
 *   102   Battery level 0~10000, sampled every 10s (read-only)
 *   3     validIndex: 16 bits, bit i = valid for RFID index i (low bit = index 0, bit 15 = index 15)
 *   4-(3+16*3)  RFID data: each index i uses 3 registers (4+i*3, 4+i*3+1, 4+i*3+2):
 *         uid_high, uid_low, signal_strength_battery (packed)
 *   103   Set volume (0~30)
 *   104   Heartbeat (0~65535)
 */
struct RfidEntry {
    uint32_t uid = 0;              // (uid_high << 16) | uid_low
    uint8_t signal_strength = 0;   // high 8 bits of register
    uint8_t battery = 0;           // low 8 bits of register
};

class HookWarning : public ModbusControl {
    public:
        // Holding register addresses
        static constexpr int REG_CTRL_STROBE = 0;
        static constexpr int REG_CTRL_SPEAKER_7M = 1;
        static constexpr int REG_CTRL_SPEAKER_3M = 2;
        static constexpr int REG_VALID_INDEX = 3;
        static constexpr int REG_RFID_BASE = 4;
        static constexpr int NUM_RFID_INDICES = 16;
        static constexpr int REGS_PER_RFID = 3;
        static constexpr int REG_STATUS_STROBE = 100;
        static constexpr int REG_STATUS_SPEAKER = 101;
        static constexpr int REG_BATTERY = 102;
        static constexpr int REG_VOLUME = 103;
        static constexpr int REG_HEARTBEAT = 104;
        static constexpr int REG_DISCHARGE_TIME = 105;
        static constexpr int REG_WORKMODE = 106;
        static constexpr int REG_CHARGING_STATUS = 107;
        static constexpr int REG_CHARGE_TIME = 108;
        static constexpr int REG_VOLTAGE = 109;
        static constexpr int REG_CURRENT = 110;
        static constexpr int REG_SHUTDOWN_TIME = 111;
        static constexpr int REG_STARTUP_TIME = 112;
        static constexpr int REG_ERROR_CODE = 113;
        static constexpr int REG_STM_RTC_TIME = 114;
        static constexpr int REG_STANDBY_ENABLE = 115;
        static constexpr int REG_PC_RTC_TIME = 116;

        HookWarning(const char* device, int baud, char parity, int data_bit, int stop_bit, int slave_id);
        HookWarning(const char* device, int baud, char parity, int data_bit, int stop_bit, int slave_id, uint32_t to_sec, uint32_t to_usec);
        HookWarning(const std::string& device, int baud, char parity, int data_bit, int stop_bit, int slave_id);
        HookWarning(const std::string& device, int baud, char parity, int data_bit, int stop_bit, int slave_id, uint32_t to_sec, uint32_t to_usec);
        ~HookWarning();

        bool connect();
        void disconnect();
        bool isConnected() const;

        bool run();

        bool slot_warning(int signal_index); // 0 for stop all warning, 1/-1 for 3m on/off, 2/-2 for 7m on/off

        uint16_t get_battery_level_feedback();
        uint16_t get_volume_feedback();

        /** Set volume (register 103). Clamped to 0~30. */
        bool setVolume(uint16_t level);
        /** Set heartbeat (register 104). Value 0~65535. */
        bool sendHeartbeat();
        /** Read heartbeat register value. */
        bool readHeartbeat(uint16_t& value);

        /** Trigger full status read (includes validIndex + RFID data). Same as run() loop. */
        bool refreshStatus();
        /** Get last read validIndex (bit i = valid for index i). */
        uint16_t getValidIndex() const;
        /** True if RFID index i has valid data (0 <= i < 16). */
        bool isRfidValid(int index) const;
        /** Get saved RFID entry for index i (0 <= i < 16). */
        RfidEntry getRfidEntry(int index) const;
        /** Get computed UID (uid_high << 16 | uid_low) for index i. */
        uint32_t getRfidUid(int index) const;
        /** Get signal strength (high 8 bits) for index i. */
        uint8_t getRfidSignalStrength(int index) const;
        /** Get battery (low 8 bits) for index i. */
        uint8_t getRfidBattery(int index) const;
        /** Get all valid RFID entries (index, RfidEntry) for indices where isRfidValid(i) is true. */
        std::vector<std::pair<int, RfidEntry>> getValidRfidEntries() const;

        // Additional status getters
        uint16_t getDischargeTime() const;
        uint16_t getWorkmode() const;
        uint16_t getChargingStatus() const;
        uint16_t getChargeTime() const;
        float getVoltage() const;
        float getCurrent() const;
        uint16_t getShutdownTime() const;
        uint16_t getStartupTime() const;
        uint16_t getErrorCode() const;
        uint16_t getStmRtcTime() const;
        uint16_t getStandbyEnable() const;
        uint16_t getPcRtcTime() const;

    private:

        double timeout = 10;

        bool strobe_on = false;
        bool speaker_3m_on = false;
        bool speaker_7m_on = false;
        uint16_t heartbeat_counter = 0;
        
        double timestamp = 0;
        bool strobe_on_feedback = false;
        bool speaker_on_feedback = false;
        uint16_t battery_level_feedback = 0;
        uint16_t volume_feedback = 0;
        uint16_t discharge_time_feedback = 0;
        uint16_t workmode_feedback = 0;
        uint16_t charging_status_feedback = 0;
        uint16_t charge_time_feedback = 0;
        float voltage_feedback = 0.0f;
        float current_feedback = 0.0f;
        uint16_t shutdown_time_feedback = 0;
        uint16_t startup_time_feedback = 0;
        uint16_t error_code_feedback = 0;
        uint16_t stm_rtc_time_feedback = 0;
        uint16_t standby_enable_feedback = 0;
        uint16_t pc_rtc_time_feedback = 0;
        
        std::thread running_thread_;
        bool running = false;

        mutable std::mutex rfid_mutex_;
        uint16_t valid_index_ = 0;
        RfidEntry rfid_entries_[NUM_RFID_INDICES];

        bool initializeDevice();

        // Control (write holding registers 0, 1, 2)
        bool setStrobe(bool on);
        bool setSpeaker7m(bool on);
        bool setSpeaker3m(bool on);
        /** Stop speaker: set both 7m and 3m to off. */
        bool setSpeakerOff();
        
        // Status (read holding registers 100, 101, 102)
        bool readStrobeStatus(bool& on);
        bool readSpeakerStatus(bool& on);
        bool readBatteryLevel(uint16_t& level);
        /** Read all status registers in one transaction (100, 101, 102). */
        bool readStatus();
};

#endif // HOOK_WARNING_H
