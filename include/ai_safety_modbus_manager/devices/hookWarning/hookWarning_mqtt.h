#ifndef HOOK_WARNING_MQTT_H
#define HOOK_WARNING_MQTT_H

#include <cstdint>
#include <vector>
#include <memory>
#include <string>
#include <mutex>
#include <thread>
#include <atomic>
#include <condition_variable>
#include "crane_log_client.h" 

// ==========================================
// 1. MQTT 协议数据结构定义 (严格 1 字节对齐)
// ==========================================
#pragma pack(push, 1)

// 1.1 帧头与通用结构
struct HookMqttFrameHeader {
    uint8_t header_byte;  // 帧头字节 (0xA5)
    uint8_t seq_num;      // 包序号 (0~255 循环递增)
    uint8_t content_id;   // 内容 ID (0x01~0x05)
    uint8_t data_len;     // 数据区长度 
};

// 1.2 内容 ID = 0x01: 灯光警报业务
struct FlashLightCmdData {
    uint8_t light_cmd    : 1; // Bit 0: 爆闪灯指令(1开/0关)
    uint8_t sound_7m_cmd : 1; // Bit 1: 爆闪灯7m响指令(1开/0关)
    uint8_t sound_3m_cmd : 1; // Bit 2: 爆闪灯3m响指令(1开/0关)
    uint8_t volume       : 5; // Bit 3-7: 音量(0~30档)
};

// 1.3 内容 ID = 0x02: 电池状态 (数据长度 10)
struct BmsStatusData {
    uint16_t battery_percent;      // 电量 0~10000 (10000=100%)
    uint16_t voltage_mv;           // 电压 单位: 0.01V
    uint16_t current_ma;           // 电流 单位: 0.01A
    uint16_t remain_time_min;      // 剩余时间 (min)
    uint16_t charge_full_time_min; // 充满时间 (min)
};

// 1.4 内容 ID = 0x04: 使能工作心跳 (数据长度 1)
struct EnableHeartbeatData {
    uint8_t enable; // 1-开启，0-关闭
};

// 1.5 内容 ID = 0x05: 心跳包 (数据长度 2)
struct HeartBeatData {
    uint16_t heartbeat; // 标准心跳 1Hz 递增
};

// 1.6 内容 ID = 0x06: 工作模式 (数据长度 1)
struct WorkModeData {
    uint8_t work_mode; // 0-待机, 1-工作, 2-低电量, 3-异常
};

// 1.7 内容 ID = 0x07: 使能睡眠模式 (数据长度 1)
struct EnableSleepModeData {
    uint8_t enable; // 1-开启，0-关闭
};

// 1.8 内容 ID = 0x08: 睡眠模式时间 (数据长度 4)
struct SleepScheduleData {
    uint16_t on_time;  // 开机时间 (高字节=时，低字节=分)
    uint16_t off_time; // 关机时间 (高字节=时，低字节=分)
};

// 1.9 内容 ID = 0x09: 当前时间 (数据长度 2)
struct CurrentTimeData {
    uint16_t rtc_time; // 当前时间 (高字节=时，低字节=分)
};

// 1.10 内容 ID = 0xF0: 系统错误码 (数据长度 1)
struct ErrorCodeData {
    uint8_t error_code; 
};

#pragma pack(pop)
// ==========================================


// ==========================================
// 2. 上位机服务类定义
// ==========================================
class HookWarningServer {
public:
    explicit HookWarningServer(const std::string& device_id);
    ~HookWarningServer();

    // --- 业务控制下发接口 ---
    // 0x01 设置声光报警
    bool set_flash_light(bool light_on, bool sound_7m, bool sound_3m, uint8_t volume);
    // 0x04 设置工作心跳使能
    bool set_heartbeat_enable(bool enable);
    // 0x07 设置睡眠模式使能
    bool set_sleep_mode_enable(bool enable);
    // 0x08 设置定时开关机时间
    bool set_time_schedule(uint8_t on_hour, uint8_t on_minute, uint8_t off_hour, uint8_t off_minute);
    // 0x09 设置当前时间
    bool set_current_time(uint8_t hour, uint8_t minute);

    // --- 数据获取接口 (来自缓存) ---
    FlashLightCmdData get_light_status();
    BmsStatusData get_bms_status();
    bool get_heartbeat_enable();
    uint8_t get_work_mode();
    bool get_sleep_mode_enable();
    SleepScheduleData get_sleep_schedule();
    CurrentTimeData get_current_time();
    uint8_t get_error_code();

private:
    std::string device_id_;

    // --- MQTT 客户端 ---
    std::unique_ptr<CraneLogClient> mqtt_client_;

    // --- MQTT 主题 ---
    std::string topic_cmd_;
    std::string topic_inform_;
    std::string topic_heartbeat_;

    // --- 线程与控制 ---
    std::atomic<bool> running_;   
    std::thread poll_thread_;     
    std::thread heartbeat_thread_;

    // --- 状态变量 ---
    std::atomic<uint8_t> send_seq_num_;        
    uint16_t current_heartbeat_ = 0;   

    // 保存从设备读取到的最新状态 (加锁保护)
    std::mutex data_mutex_;
    FlashLightCmdData latest_light_status_;
    BmsStatusData latest_bms_status_;
    EnableHeartbeatData latest_heartbeat_enable_;
    WorkModeData latest_work_mode_;
    EnableSleepModeData latest_sleep_mode_enable_;
    SleepScheduleData latest_sleep_schedule_;
    CurrentTimeData latest_current_time_;
    ErrorCodeData latest_error_code_;

    // 应答等待（用于 set_xxx 方法阻塞等待下位机回包）
    std::mutex response_mutex_;
    std::condition_variable response_cv_;
    uint8_t responded_content_id_ = 0;

    // --- 核心方法 ---
    void init_mqtt();

    void on_topic_inform(const std::string& topic, const std::string& payload);
    void parse_mqtt_frame(const std::string& topic, const uint8_t* raw_data, size_t length);
    
    // 严格依据文档的 CRC16-CCITT
    uint16_t calculate_crc16(const uint8_t* data, size_t length);
    bool verify_crc16(const uint8_t* data, size_t length, uint16_t received_crc);

    // 发送指令基础函数
    void send_command(uint8_t content_id, const uint8_t* data, uint8_t data_len);
    void send_inquiry(uint8_t content_id); 
    
    // 阻塞发送并等待应答辅助函数
    bool send_command_and_wait(uint8_t content_id, const uint8_t* data, uint8_t data_len, int timeout_ms = 2000);

    // 独立线程执行的定时任务
    void poll_task();
    void heartbeat_task();
};

#endif // HOOK_WARNING_MQTT_H
