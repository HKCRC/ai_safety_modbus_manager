#include "devices/hookWarning/hookWarning_mqtt.h"
#include <cstring>
#include <iostream>
#include <chrono>
#include <arpa/inet.h>
#include <unistd.h>


#define FRAME_HEADER_BYTE 0xA5

// ==========================================
// 构造函数与初始化
// ==========================================
HookWarningServer::HookWarningServer(const std::string& device_id)
    : device_id_(device_id),
      send_seq_num_(0),
      current_heartbeat_(0)
{
  
    topic_cmd_       = "ai_safety/" + device_id_ + "/command/device_manager/hook";
    topic_inform_    = "ai_safety/" + device_id_ + "/inform/hook/device_manager";
    topic_heartbeat_ = "ai_safety/" + device_id_ + "/heartbeat/device_manager/hook";
    
    // 初始化数据缓存
    std::memset(&latest_light_status_, 0, sizeof(latest_light_status_));
    std::memset(&latest_bms_status_, 0, sizeof(latest_bms_status_));
    std::memset(&latest_heartbeat_enable_, 0, sizeof(latest_heartbeat_enable_));
    std::memset(&latest_work_mode_, 0, sizeof(latest_work_mode_));
    std::memset(&latest_sleep_mode_enable_, 0, sizeof(latest_sleep_mode_enable_));
    std::memset(&latest_sleep_schedule_, 0, sizeof(latest_sleep_schedule_));
    std::memset(&latest_current_time_, 0, sizeof(latest_current_time_));
    std::memset(&latest_error_code_, 0, sizeof(latest_error_code_));
    last_rx_time_ = std::chrono::steady_clock::now();

    init_mqtt();

    std::cout << "[HookWarningServer] Initialized for device: " << device_id_ << std::endl;
}

HookWarningServer::~HookWarningServer() {
    running_ = false;
    if (started_.load()) {
        if (poll_thread_.joinable()) poll_thread_.join();
        if (heartbeat_thread_.joinable()) heartbeat_thread_.join();
    }
}

void HookWarningServer::start() {
    if (started_.exchange(true)) {
        return;
    }
    running_ = true;
    poll_thread_ = std::thread(&HookWarningServer::poll_task, this);
    heartbeat_thread_ = std::thread(&HookWarningServer::heartbeat_task, this);
}

void HookWarningServer::init_mqtt() {
    std::lock_guard<std::mutex> lock(mqtt_mutex_);

    MqttConfig cloud_cfg;
    cloud_cfg.host = "210.0.159.242";
    cloud_cfg.port = 1883;
    cloud_cfg.username = "hkcrctest";
    cloud_cfg.password = "crcHK3130";

    char hostname[256];
    gethostname(hostname, sizeof(hostname));
    cloud_cfg.client_id = std::string("crane_cloud_HookWarning_") + hostname + "/" + MqttClient::getLocalIP();

    mqtt_client_ = std::make_unique<MqttClient>();
    mqtt_client_->init(cloud_cfg);

    auto inform_cb = [this](const std::string& topic, const std::string& payload) {
        this->on_topic_inform(topic, payload);
    };

    mqtt_client_->subscribe(topic_inform_, inform_cb, 1);
}

bool HookWarningServer::is_connected() const {
    if (!mqtt_client_) return false;
    return mqtt_client_->connected();
}

void HookWarningServer::on_topic_inform(const std::string& topic, const std::string& payload) {
    parse_mqtt_frame(topic, reinterpret_cast<const uint8_t*>(payload.data()), payload.length());
}

void HookWarningServer::parse_mqtt_frame(const std::string& topic, const uint8_t* raw_data, size_t raw_length) {
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        last_rx_time_ = std::chrono::steady_clock::now();
    }

    if (raw_length < 6) return; // 帧头(4) + CRC(2)
    
    const HookMqttFrameHeader* header = reinterpret_cast<const HookMqttFrameHeader*>(raw_data);
    
    // 校验总长度是否匹配: 帧头(4) + 数据区(data_len) + CRC(2)
    size_t expected_len = sizeof(HookMqttFrameHeader) + header->data_len + 2;
    if (raw_length != expected_len) {
        std::cerr << "[HookWarningServer] Length mismatch. Expected: " << expected_len << ", Actual: " << raw_length << "\n";
        return;
    }

    // 提取帧尾的 CRC16 (低字节在前，高字节在后)
    uint16_t received_crc = raw_data[expected_len - 2] | (raw_data[expected_len - 1] << 8);
    
    // 校验范围：除去最后2个字节的 CRC
    if (!verify_crc16(raw_data, expected_len - 2, received_crc)) {
        static auto last_print_time = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_print_time).count() >= 5) { // 限制为最多 5 秒打印一次
            std::cerr << "[HookWarningServer] CRC16 Validation Failed!\n";
            last_print_time = now;
        }
        return;
    }

    const uint8_t* data_ptr = raw_data + sizeof(HookMqttFrameHeader);
    uint8_t content_id = header->content_id;
    
    // 错误帧处理
    if (content_id & 0x80) {
        uint8_t error_cmd = content_id & 0x7F;
        uint8_t error_code = data_ptr[0];
        static auto last_err_print = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_err_print).count() >= 5) {
            std::cerr << "[HookWarningServer] Received error frame for CMD_ID 0x" << std::hex << (int)error_cmd 
                      << ", error_code: " << std::dec << (int)error_code << "\n";
            last_err_print = now;
        }
        {
            std::lock_guard<std::mutex> lk(response_mutex_);
            responded_content_id_ = content_id; // 保存带 0x80 标志的 ID 以唤醒并通知失败
        }
        response_cv_.notify_all();
        return;
    }

    // 正常应答帧处理 (读应答有数据，写应答无数据)
    if (header->data_len > 0) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        switch (content_id) {
            case 0x01: { // 灯光警报状态
                if (header->data_len == sizeof(FlashLightCmdData)) {
                    std::memcpy(&latest_light_status_, data_ptr, sizeof(FlashLightCmdData));
                }
                break;
            }
            case 0x02: { // 电池状态
                if (header->data_len == sizeof(BmsStatusData)) {
                    const BmsStatusData* net_data = reinterpret_cast<const BmsStatusData*>(data_ptr);
                    latest_bms_status_.battery_percent = ntohs(net_data->battery_percent);
                    latest_bms_status_.voltage_mv = ntohs(net_data->voltage_mv);
                    latest_bms_status_.current_ma = ntohs(net_data->current_ma);
                    latest_bms_status_.remain_time_min = ntohs(net_data->remain_time_min);
                    latest_bms_status_.charge_full_time_min = ntohs(net_data->charge_full_time_min);
                }
                break;
            }
            case 0x04: { // 使能工作心跳
                if (header->data_len == sizeof(heart_beat_en_t)) {
                    std::memcpy(&latest_heartbeat_enable_, data_ptr, sizeof(heart_beat_en_t));
                }
                break;
            }
            case 0x05: { // 心跳包应答
                break; 
            }
            case 0x06: { // 工作模式
                if (header->data_len == sizeof(WorkModeData)) {
                    std::memcpy(&latest_work_mode_, data_ptr, sizeof(WorkModeData));
                }
                break;
            }
            case 0x07: { // 使能睡眠模式
                if (header->data_len == sizeof(EnableSleepModeData)) {
                    std::memcpy(&latest_sleep_mode_enable_, data_ptr, sizeof(EnableSleepModeData));
                }
                break;
            }
            case 0x08: { // 睡眠模式时间
                if (header->data_len == sizeof(SleepScheduleData)) {
                    const SleepScheduleData* net_data = reinterpret_cast<const SleepScheduleData*>(data_ptr);
                    latest_sleep_schedule_.on_time = ntohs(net_data->on_time);
                    latest_sleep_schedule_.off_time = ntohs(net_data->off_time);
                }
                break;
            }
            case 0x09: { // 当前时间
                if (header->data_len == sizeof(CurrentTimeData)) {
                    const CurrentTimeData* net_data = reinterpret_cast<const CurrentTimeData*>(data_ptr);
                    latest_current_time_.rtc_time = ntohs(net_data->rtc_time);
                }
                break;
            }
            case 0xF0: { // 系统错误码
                if (header->data_len == sizeof(ErrorCodeData)) {
                    std::memcpy(&latest_error_code_, data_ptr, sizeof(ErrorCodeData));
                }
                break;
            }
            default:
                std::cout << "[HookWarningServer] Unhandled Content ID: 0x" << std::hex << (int)content_id << std::dec << "\n";
                break;
        }
    }

    {
        std::lock_guard<std::mutex> lk(response_mutex_);
        responded_content_id_ = content_id;
    }
    
    static auto last_resp_print = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::seconds>(now - last_resp_print).count() >= 5) {
        std::cout << "[HookWarningServer] Got 0x" << std::hex << (int)content_id << std::dec
                  << " response from " << topic << std::endl;
        last_resp_print = now;
    }
    
    response_cv_.notify_all();
}

// ==========================================
// 数据下发与发送封装
// ==========================================
bool HookWarningServer::send_command(uint8_t content_id, const uint8_t* data, uint8_t data_len) {
    std::lock_guard<std::mutex> lock(mqtt_mutex_);
    if (!mqtt_client_) return false;

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

    // 计算 CRC16，并拼接到末尾 (低字节在前，高字节在后)
    uint16_t crc = calculate_crc16(buffer.data(), total_len - 2);
    buffer[total_len - 2] = crc & 0xFF;        // 低字节
    buffer[total_len - 1] = (crc >> 8) & 0xFF; // 高字节

    std::string payload_str(reinterpret_cast<char*>(buffer.data()), total_len);
    if (content_id == 0x05) {
        return mqtt_client_->publish(topic_heartbeat_, payload_str, 0, false);
    } else {
        return mqtt_client_->publish(topic_cmd_, payload_str, 2, false);
    }
}

bool HookWarningServer::send_inquiry(uint8_t content_id) {
    return send_command(content_id, nullptr, 0);
}

bool HookWarningServer::send_command_and_wait(uint8_t content_id, const uint8_t* data, uint8_t data_len, int timeout_ms) {
    {
        std::lock_guard<std::mutex> lk(response_mutex_);
        responded_content_id_ = 0;
    }
    
    send_command(content_id, data, data_len);
    std::cout << "[HookWarningServer] 0x" << std::hex << (int)content_id << std::dec 
              << " command sent, waiting for response..." << std::endl;

    std::unique_lock<std::mutex> lk(response_mutex_);
    bool ok = response_cv_.wait_for(lk, std::chrono::milliseconds(timeout_ms), [this, content_id]{ 
        return responded_content_id_ == content_id || responded_content_id_ == (content_id | 0x80); 
    });
    
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 限制频率防粘包
    return ok && (responded_content_id_ == content_id);
}

// 0x01 控制灯光与喇叭
bool HookWarningServer::set_flash_light(bool light_on, bool sound_7m, bool sound_3m, uint8_t volume) {
    FlashLightCmdData cmd;
    std::memset(&cmd, 0, sizeof(cmd));
    cmd.light_cmd = light_on ? 1 : 0;
    cmd.sound_7m_cmd = sound_7m ? 1 : 0;
    cmd.sound_3m_cmd = sound_3m ? 1 : 0;
    cmd.volume = volume & 0x1F;

    for (int i = 0; i < 3; ++i) {
        if (send_command_and_wait(0x01, reinterpret_cast<const uint8_t*>(&cmd), sizeof(FlashLightCmdData))) {
            return true;
        }
    }
    return false;
}

// 0x04 控制心跳使能
bool HookWarningServer::set_heartbeat_enable(bool enable) {
    heart_beat_en_t cmd;
    cmd.heartbeat_en = enable ? 1 : 0;

    for (int i = 0; i < 3; ++i) {
        if (send_command_and_wait(0x04, reinterpret_cast<const uint8_t*>(&cmd), sizeof(heart_beat_en_t))) {
            return true;
        }
    }
    return false;
}

// 0x07 控制睡眠模式使能
bool HookWarningServer::set_sleep_mode_enable(bool enable) {
    EnableSleepModeData cmd;
    cmd.enable = enable ? 1 : 0;

    for (int i = 0; i < 3; ++i) {
        if (send_command_and_wait(0x07, reinterpret_cast<const uint8_t*>(&cmd), sizeof(EnableSleepModeData))) {
            return true;
        }
    }
    return false;
}

// 0x08 控制系统作息时间
bool HookWarningServer::set_time_schedule(uint8_t on_hour, uint8_t on_minute, uint8_t off_hour, uint8_t off_minute) {
    SleepScheduleData cmd;
    uint16_t host_on_time  = (on_hour << 8) | on_minute;
    uint16_t host_off_time = (off_hour << 8) | off_minute;

    cmd.on_time  = htons(host_on_time);
    cmd.off_time = htons(host_off_time);

    for (int i = 0; i < 3; ++i) {
        if (send_command_and_wait(0x08, reinterpret_cast<const uint8_t*>(&cmd), sizeof(SleepScheduleData))) {
            return true;
        }
    }
    return false;
}

// 0x09 设置当前时间
bool HookWarningServer::set_current_time(uint8_t hour, uint8_t minute) {
    CurrentTimeData cmd;
    uint16_t host_time = (hour << 8) | minute;
    cmd.rtc_time = htons(host_time);

    for (int i = 0; i < 3; ++i) {
        if (send_command_and_wait(0x09, reinterpret_cast<const uint8_t*>(&cmd), sizeof(CurrentTimeData))) {
            return true;
        }
    }
    return false;
}

// ==========================================
// 后台定时任务
// ==========================================
void HookWarningServer::poll_task() {
    int counter_100ms = 0;
    int consecutive_publish_failures = 0;

    while (running_) {
        // 电池状态 (0x02) 每 10 秒 (100 * 100ms) 查询一次
        if (counter_100ms % 100 == 0) {
            bool ok = send_inquiry(0x02);
            if (!ok) consecutive_publish_failures++; else consecutive_publish_failures = 0;
        }

        std::chrono::steady_clock::time_point rx_time;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            rx_time = last_rx_time_;
        }
        auto now = std::chrono::steady_clock::now();
        bool receive_timeout = std::chrono::duration_cast<std::chrono::seconds>(now - rx_time).count() > 15;

        if (consecutive_publish_failures >= 5 || receive_timeout) {
            std::cout << "[HookWarningServer] MQTT disconnected or deaf (timeout: "
                      << receive_timeout << ", fails: " << consecutive_publish_failures
                      << "), attempting to re-initialize..." << std::endl;
            init_mqtt();
            consecutive_publish_failures = 0;
            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                last_rx_time_ = std::chrono::steady_clock::now();
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        counter_100ms++;

        if (counter_100ms >= 100) {
            counter_100ms = 0;
        }
    }
}

void HookWarningServer::heartbeat_task() {
    while (running_) {
        heart_beat_t hb;
        hb.heartbeat = current_heartbeat_;
        if (++current_heartbeat_ > 255) current_heartbeat_ = 0;
        send_command(0x05, reinterpret_cast<const uint8_t*>(&hb), sizeof(heart_beat_t));
        
        // 每 1 秒发送一次心跳 (10 * 100ms)
        for (int i = 0; i < 10 && running_; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
}

uint16_t HookWarningServer::calculate_crc16(const uint8_t* data, size_t length) {
    uint16_t crc = 0x0000; 
    
    for (size_t i = 0; i < length; ++i) {
        crc ^= (uint16_t)(data[i]) << 8; // 当前字节移到高 8 位并异或
        
        for (uint8_t j = 0; j < 8; ++j) {
            if (crc & 0x8000) { // 判断最高位
                crc = (uint16_t)((crc << 1) ^ 0x1021);
            } else {
                crc = (uint16_t)(crc << 1);
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
FlashLightCmdData HookWarningServer::get_light_status() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return latest_light_status_;
}

BmsStatusData HookWarningServer::get_bms_status() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return latest_bms_status_;
}

bool HookWarningServer::get_heartbeat_enable() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return latest_heartbeat_enable_.heartbeat_en == 1;
}

uint8_t HookWarningServer::get_work_mode() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return latest_work_mode_.work_mode;
}

bool HookWarningServer::get_sleep_mode_enable() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return latest_sleep_mode_enable_.enable == 1;
}

SleepScheduleData HookWarningServer::get_sleep_schedule() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return latest_sleep_schedule_;
}

CurrentTimeData HookWarningServer::get_current_time() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return latest_current_time_;
}

uint8_t HookWarningServer::get_error_code() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return latest_error_code_.error_code;
}
