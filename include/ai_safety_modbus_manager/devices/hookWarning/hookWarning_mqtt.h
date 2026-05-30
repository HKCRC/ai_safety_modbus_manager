#ifndef HOOK_WARNING_MQTT_H
#define HOOK_WARNING_MQTT_H

#include <cstdint>
#include <vector>
#include <memory>
#include <string>
#include <mutex>
#include <thread>
#include <atomic>
#include "crane_log_client.h" 

// ==========================================
// 1. MQTT 协议数据结构定义 (严格 1 字节对齐)
// ==========================================
#pragma pack(push, 1)

// 1.1 帧头与通用结构
struct HookMqttFrameHeader {
    uint8_t header_byte;  // 帧头字节 (0xAA)
    uint8_t content_id;   // 内容 ID (0x01~0x05)
    uint8_t seq_num;      // 包序号 (0~255 循环递增)
    uint8_t data_len;     // 数据区长度 
};

// 1.2 内容 ID = 0x01: 灯光警报业务
struct FlashLightCmdData {
    uint8_t light_cmd    : 1; // Bit 0: 爆闪灯指令(1开/0关)
    uint8_t sound_7m_cmd : 1; // Bit 1: 爆闪灯7m响指令(1开/0关)
    uint8_t sound_3m_cmd : 1; // Bit 2: 爆闪灯3m响指令(1开/0关)
    uint8_t volume       : 5; // Bit 3-7: 音量(0~30档)
};

struct FlashLightStatusData {
    uint8_t light_en : 1;
    uint8_t sound_en : 1;
    uint8_t reserved : 1;
    uint8_t volume   : 5;
};

// 1.3 内容 ID = 0x02: 电池状态 (数据长度 10)
struct BmsStatusData {
    uint16_t battery_percent;      // 电量 0~10000 (10000=100%)
    uint16_t voltage_mv;           // 电压 单位: 0.01V
    uint16_t current_ma;           // 电流 单位: 0.01A
    uint16_t remain_time_min;      // 剩余时间 (min)
    uint16_t charge_full_time_min; // 充满时间 (min)
};

// 1.4 内容 ID = 0x03: 系统相关 (数据长度 3)
struct SysStatusData {
    uint8_t is_standby; // 1-开启待机，0-关闭
    uint8_t work_mode;  // 0-待机, 1-工作, 2-低电量, 3-异常
    uint8_t error_code; // 错误码
};

// 1.5 内容 ID = 0x04: 时间相关 (数据长度 6)
struct SysScheduleData {
    uint16_t off_time; // 关机时间 (高字节=时，低字节=分)
    uint16_t on_time;  // 开机时间
    uint16_t rtc_time; // rtc时间
};

// 1.6 内容 ID = 0x05: 心跳包 (数据长度 1)
struct HeartBeatData {
    uint8_t heartbeat;
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
    void set_flash_light(bool light_on, bool sound_7m, bool sound_3m, uint8_t volume);
    // 0x03 设置系统模式 (待机位、工作模式)
    void set_sys_mode(bool is_standby, uint8_t work_mode);
    // 0x04 设置定时开关机时间 (传入小时和分钟，例如 21, 0 表示 21:00)
    void set_time_schedule(uint8_t off_hour, uint8_t off_minute, uint8_t on_hour, uint8_t on_minute);

    // --- 数据获取接口  ---
    BmsStatusData get_bms_status();
    FlashLightStatusData get_light_status();
    SysStatusData get_sys_status();
    SysScheduleData get_schedule_status();

private:
    std::string device_id_;

    // --- MQTT 客户端 ---
    std::unique_ptr<CraneLogClient> mqtt_client_;

    // --- MQTT 主题 ---
    std::string topic_cmd_;
    std::string topic_inform_;
    std::string topic_upload_;

    // --- 线程与控制 ---
    std::atomic<bool> running_;   
    std::thread poll_thread_;     
    std::thread heartbeat_thread_;

    // --- 状态变量 ---
    std::atomic<uint8_t> send_seq_num_;        
    uint8_t current_heartbeat_;   

    // 保存从设备读取到的最新状态 (加锁保护)
    std::mutex data_mutex_;
    BmsStatusData latest_bms_status_;
    FlashLightStatusData latest_light_status_;
    SysStatusData latest_sys_status_;
    SysScheduleData latest_schedule_status_;

    // --- 核心方法 ---
    void init_mqtt();

    void on_topic_inform(const std::string& topic, const std::string& payload);
    void on_topic_upload(const std::string& topic, const std::string& payload);
    void parse_mqtt_frame(const std::string& topic, const uint8_t* raw_data, size_t length);
    
    // 严格依据文档的 CRC16-CCITT
    uint16_t calculate_crc16(const uint8_t* data, size_t length);
    bool verify_crc16(const uint8_t* data, size_t length, uint16_t received_crc);

    // 发送指令基础函数
    void send_command(uint8_t content_id, const uint8_t* data, uint8_t data_len);
    void send_inquiry(uint8_t content_id); 

    // 独立线程执行的定时任务
    void poll_task();
    void heartbeat_task();
};

#endif // HOOK_WARNING_MQTT_H