#ifndef MODBUS_CONFIG_H
#define MODBUS_CONFIG_H

#include <string>

struct ModbusConfig {
    std::string crane_type = "flat";
    std::string x86_ip = "192.168.61.2";
    std::string trolley_ip = "192.168.61.36";
    std::string cab_bridge_ip = "192.168.61.67";
    std::string trolley_bridge_ip = "192.168.61.62";

    int trolley_port = 502;
    int trolley_slave = 1;

    std::string hook_mqtt_device_id;

    std::string encoder_dev = "/dev/ttyS1";
    std::string encoder_ip;
    int encoder_port = 502;
    int encoder_baud = 9600;
    char encoder_parity = 'E';
    int encoder_data_bit = 8;
    int encoder_stop_bit = 1;
    int encoder_slave = 1;

    float laser_scale_m = 0.001f;
    float laser1_tilt_deg = 4.0f;
    float laser2_tilt_deg = 12.0f;

    float encoder_k = 1.0f;
    float encoder_b = 0.0f;

    int loop_ms = 1000;
    
    // 从文件加载配置，成功返回true
    bool loadFromFile(const std::string& path);
};

#endif // MODBUS_CONFIG_H
