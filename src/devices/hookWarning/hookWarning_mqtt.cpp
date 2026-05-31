#include "devices/hookWarning/hookWarning_mqtt.h"
#include <cstring>
#include <iostream>
#include <chrono>
#include <arpa/inet.h> 


#define FRAME_HEADER_BYTE 0xAA

// ==========================================
// 构造函数与初始化
// ==========================================
HookWarningServer::HookWarningServer(const std::string& device_id)
    : device_id_(device_id),
      running_(true),
      send_seq_num_(0),
      current_heartbeat_(0)
{
    // 动态拼接 Topic，匹配文档要求: /ai_safety/<ais001>/...
    topic_cmd_    = "/ai_safety/" + device_id_ + "/control/command/device_manager/hook";
    topic_inform_ = "/ai_safety/" + device_id_ + "/control/inform/hook/device_manager";
    topic_upload_ = "/ai_safety/" + device_id_ + "/info/upload/hook/device_manager";
    
    // 初始化数据缓存
    std::memset(&latest_bms_status_, 0, sizeof(latest_bms_status_));
    std::memset(&latest_light_status_, 0, sizeof(latest_light_status_));
    std::memset(&latest_sys_status_, 0, sizeof(latest_sys_status_));
    std::memset(&latest_schedule_status_, 0, sizeof(latest_schedule_status_));

    init_mqtt();

    poll_thread_ = std::thread(&HookWarningServer::poll_task, this);
    heartbeat_thread_ = std::thread(&HookWarningServer::heartbeat_task, this);

    std::cout << "[HookWarningServer] Initialized for device: " << device_id_ << std::endl;
}

HookWarningServer::~HookWarningServer() {
    running_ = false;
    if (poll_thread_.joinable()) poll_thread_.join();
    if (heartbeat_thread_.joinable()) heartbeat_thread_.join();
}

void HookWarningServer::init_mqtt() {
    mqtt_client_ = std::make_unique<CraneLogClient>("HookWarning");
    mqtt_client_->enable_local_mqtt(false);

    auto inform_cb = [this](const std::string& topic, const std::string& payload) {
        this->on_topic_inform(topic, payload);
    };
    auto upload_cb = [this](const std::string& topic, const std::string& payload) {
        this->on_topic_upload(topic, payload);
    };

    mqtt_client_->mqtt_cloud_subscribe(topic_inform_, inform_cb, 1);
    mqtt_client_->mqtt_cloud_subscribe(topic_upload_, upload_cb, 1);
}

void HookWarningServer::on_topic_inform(const std::string& topic, const std::string& payload) {
    parse_mqtt_frame(topic, reinterpret_cast<const uint8_t*>(payload.data()), payload.length());
}

void HookWarningServer::on_topic_upload(const std::string& topic, const std::string& payload) {
    parse_mqtt_frame(topic, reinterpret_cast<const uint8_t*>(payload.data()), payload.length());
}

void HookWarningServer::parse_mqtt_frame(const std::string& topic, const uint8_t* raw_data, size_t raw_length) {
    if (raw_length < 6) return; // 至少: 帧头(4) + CRC(2)
    
    const HookMqttFrameHeader* header = reinterpret_cast<const HookMqttFrameHeader*>(raw_data);
    
    // 校验总长度是否匹配: 帧头(4) + 数据区(data_len) + CRC(2)
    size_t expected_len = sizeof(HookMqttFrameHeader) + header->data_len + 2;
    if (raw_length != expected_len) {
        std::cerr << "[HookWarningServer] Length mismatch. Expected: " << expected_len << ", Actual: " << raw_length << "\n";
        return;
    }

    // 提取帧尾的 CRC16 
    uint16_t received_crc = (raw_data[expected_len - 2] << 8) | raw_data[expected_len - 1];
    
    // 校验范围：除去最后2个字节的 CRC
    if (!verify_crc16(raw_data, expected_len - 2, received_crc)) {
        std::cerr << "[HookWarningServer] CRC16 Validation Failed!\n";
        return;
    }

    const uint8_t* data_ptr = raw_data + sizeof(HookMqttFrameHeader);
    uint8_t content_id = header->content_id & 0x7F; // 屏蔽掉应答标志 0x80

    std::lock_guard<std::mutex> lock(data_mutex_);
    switch (content_id) {
        case 0x01: { // 0x81: 灯光警报状态
            if (header->data_len == sizeof(FlashLightStatusData)) {
                std::memcpy(&latest_light_status_, data_ptr, sizeof(FlashLightStatusData));
            }
            break;
        }
        case 0x02: { // 0x82: 电池状态
            if (header->data_len == sizeof(BmsStatusData)) {
                // 不能用 memcpy，必须处理大端转小端
                const BmsStatusData* net_data = reinterpret_cast<const BmsStatusData*>(data_ptr);
                latest_bms_status_.battery_percent = ntohs(net_data->battery_percent);
                latest_bms_status_.voltage_mv = ntohs(net_data->voltage_mv);
                latest_bms_status_.current_ma = ntohs(net_data->current_ma);
                latest_bms_status_.remain_time_min = ntohs(net_data->remain_time_min);
                latest_bms_status_.charge_full_time_min = ntohs(net_data->charge_full_time_min);
            }
            break;
        }
        case 0x03: { // 0x83: 系统状态
            if (header->data_len == sizeof(SysStatusData)) {
                std::memcpy(&latest_sys_status_, data_ptr, sizeof(SysStatusData)); // 全是 1 字节，可以直接 memcpy
            }
            break;
        }
        case 0x04: { // 0x84: 时间相关状态
            if (header->data_len == sizeof(SysScheduleData)) {
                // 不能用 memcpy，必须处理大端转小端
                const SysScheduleData* net_data = reinterpret_cast<const SysScheduleData*>(data_ptr);
                latest_schedule_status_.off_time = ntohs(net_data->off_time);
                latest_schedule_status_.on_time = ntohs(net_data->on_time);
                latest_schedule_status_.rtc_time = ntohs(net_data->rtc_time);
            }
            break;
        }
        case 0x05: { // 0x85: 心跳包应答
            break; 
        }
        default:
            std::cout << "[HookWarningServer] Unhandled Content ID: 0x" << std::hex << (int)header->content_id << std::dec << "\n";
            break;
    }
}

// ==========================================
// 数据下发与发送封装
// ==========================================
void HookWarningServer::send_command(uint8_t content_id, const uint8_t* data, uint8_t data_len) {
    size_t total_len = sizeof(HookMqttFrameHeader) + data_len + 2;
    std::vector<uint8_t> buffer(total_len, 0);

    HookMqttFrameHeader* header = reinterpret_cast<HookMqttFrameHeader*>(buffer.data());
    header->header_byte = FRAME_HEADER_BYTE; 
    header->content_id = content_id;
    header->seq_num = send_seq_num_++; 
    header->data_len = data_len;

    if (data != nullptr && data_len > 0) {
        std::memcpy(buffer.data() + sizeof(HookMqttFrameHeader), data, data_len);
    }

    // 计算 CRC16，并拼接到末尾
    uint16_t crc = calculate_crc16(buffer.data(), total_len - 2);
    buffer[total_len - 2] = (crc >> 8) & 0xFF; // 高字节
    buffer[total_len - 1] = crc & 0xFF;        // 低字节

    std::string payload_str(reinterpret_cast<char*>(buffer.data()), total_len);
    mqtt_client_->mqtt_publish(topic_cmd_, payload_str, 1, false);
}

void HookWarningServer::send_inquiry(uint8_t content_id) {
    send_command(content_id, nullptr, 0);
}

// 0x01 控制灯光与喇叭
void HookWarningServer::set_flash_light(bool light_on, bool sound_7m, bool sound_3m, uint8_t volume) {
    FlashLightCmdData cmd;
    std::memset(&cmd, 0, sizeof(cmd));
    cmd.light_cmd = light_on ? 1 : 0;
    cmd.sound_7m_cmd = sound_7m ? 1 : 0;
    cmd.sound_3m_cmd = sound_3m ? 1 : 0;
    cmd.volume = volume & 0x1F; 

    send_command(0x01, reinterpret_cast<const uint8_t*>(&cmd), sizeof(FlashLightCmdData));
    
    // 强制阻塞 100ms，限制外部调用的频率，防止连续发包导致下位机粘包或死机
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

// 0x03 控制系统模式
void HookWarningServer::set_sys_mode(bool is_standby, uint8_t work_mode) {
    SysStatusData cmd;
    std::memset(&cmd, 0, sizeof(cmd));
    cmd.is_standby = is_standby ? 1 : 0;
    cmd.work_mode = work_mode;
    cmd.error_code = 0; 
    
    send_command(0x03, reinterpret_cast<const uint8_t*>(&cmd), sizeof(SysStatusData));
    
    // 强制阻塞 100ms
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

// 0x04 控制系统作息时间
void HookWarningServer::set_time_schedule(uint8_t off_hour, uint8_t off_minute, uint8_t on_hour, uint8_t on_minute) {
    SysScheduleData cmd;
    std::memset(&cmd, 0, sizeof(cmd));
    
    // 主机组装数据（拼接时是主机小端序）
    uint16_t host_off_time = (off_hour << 8) | off_minute;
    uint16_t host_on_time  = (on_hour << 8) | on_minute;
    
    // 发送前必须转为网络大端序
    cmd.off_time = htons(host_off_time);
    cmd.on_time  = htons(host_on_time);
    cmd.rtc_time = 0; 
    
    send_command(0x04, reinterpret_cast<const uint8_t*>(&cmd), sizeof(SysScheduleData));
    
    // 强制阻塞 100ms
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

// ==========================================
// 后台定时任务
// ==========================================
void HookWarningServer::poll_task() {
    int counter_100ms = 0; // 用一个 100ms 为单位的计数器来管理不同任务的频率
    
    while (running_) {
        // 系统状态 (0x03) 和 作息时间 (0x04) 每 2 秒 (20 * 100ms) 查询一次
        if (counter_100ms % 20 == 0) {
            send_inquiry(0x03);
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 错开 100ms 防止粘包
            send_inquiry(0x04);
        }
        
        // 电池状态 (0x02) 每 5 秒 (50 * 100ms) 查询一次
        if (counter_100ms % 50 == 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 错开 100ms 防止粘包
            send_inquiry(0x02);
        }

        // 每次循环睡 100ms，并把计数器加 1
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        counter_100ms++;
        
        // 防止计数器溢出（虽然 int 很大，但在长时间运行中找个公倍数清零更安全）
        // 20 和 50 的最小公倍数是 100 (代表 10 秒)
        if (counter_100ms >= 100) {
            counter_100ms = 0;
        }
    }
}

void HookWarningServer::heartbeat_task() {
    while (running_) {
        HeartBeatData hb;
        hb.heartbeat = current_heartbeat_++; 
        send_command(0x05, reinterpret_cast<const uint8_t*>(&hb), sizeof(HeartBeatData));
        
        for (int i = 0; i < 300 && running_; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
}

// ==========================================
// CRC16-CCITT 算法
// ==========================================
uint16_t HookWarningServer::calculate_crc16(const uint8_t* data, size_t length) {
    uint16_t crc = 0x0000;
    for (size_t i = 0; i < length; ++i) {
        crc ^= ((uint16_t)data[i] << 8);
        for (uint8_t j = 0; j < 8; ++j) {
            if (crc & 0x8000) {
                crc = (uint16_t)((crc << 1) ^ 0x1021);
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

bool HookWarningServer::verify_crc16(const uint8_t* data, size_t length, uint16_t received_crc) {
    return calculate_crc16(data, length) == received_crc;
}

// ==========================================
// 数据获取接口 (Getters)
// ==========================================
BmsStatusData HookWarningServer::get_bms_status() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return latest_bms_status_;
}

FlashLightStatusData HookWarningServer::get_light_status() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return latest_light_status_;
}

SysStatusData HookWarningServer::get_sys_status() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return latest_sys_status_;
}

SysScheduleData HookWarningServer::get_schedule_status() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return latest_schedule_status_;
}
