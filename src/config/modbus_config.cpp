#include "config/modbus_config.h"

#include <fstream>
#include <iostream>
#include <jsoncpp/json/json.h>

bool ModbusConfig::loadFromFile(const std::string& path) {
    std::ifstream ifs(path);
    if (!ifs.is_open()) {
        std::cerr << "Failed to open config file: " << path << std::endl;
        return false;
    }

    Json::Value root;
    Json::CharReaderBuilder builder;
    std::string errs;
    if (!Json::parseFromStream(builder, ifs, &root, &errs)) {
        std::cerr << "Failed to parse JSON config: " << errs << std::endl;
        return false;
    }

    if (root.isMember("crane_type") && root["crane_type"].isString()) crane_type = root["crane_type"].asString();
    if (root.isMember("x86_ip") && root["x86_ip"].isString()) x86_ip = root["x86_ip"].asString();
    if (root.isMember("trolley_ip") && root["trolley_ip"].isString()) trolley_ip = root["trolley_ip"].asString();
    if (root.isMember("cab_bridge_ip") && root["cab_bridge_ip"].isString()) cab_bridge_ip = root["cab_bridge_ip"].asString();
    if (root.isMember("trolley_bridge_ip") && root["trolley_bridge_ip"].isString()) trolley_bridge_ip = root["trolley_bridge_ip"].asString();

    if (root.isMember("trolley_port") && root["trolley_port"].isInt()) trolley_port = root["trolley_port"].asInt();
    if (root.isMember("trolley_slave") && root["trolley_slave"].isInt()) trolley_slave = root["trolley_slave"].asInt();


    if (root.isMember("hook_mqtt_device_id") && root["hook_mqtt_device_id"].isString()) {
        hook_mqtt_device_id = root["hook_mqtt_device_id"].asString();
    }

    if (root.isMember("encoder_dev") && root["encoder_dev"].isString()) encoder_dev = root["encoder_dev"].asString();
    if (root.isMember("encoder_ip") && root["encoder_ip"].isString()) encoder_ip = root["encoder_ip"].asString();
    if (root.isMember("encoder_port") && root["encoder_port"].isInt()) encoder_port = root["encoder_port"].asInt();
    if (root.isMember("encoder_baud") && root["encoder_baud"].isInt()) encoder_baud = root["encoder_baud"].asInt();
    if (root.isMember("encoder_parity") && root["encoder_parity"].isString()) {
        std::string p = root["encoder_parity"].asString();
        if (!p.empty()) encoder_parity = p[0];
    }
    if (root.isMember("encoder_data_bit") && root["encoder_data_bit"].isInt()) encoder_data_bit = root["encoder_data_bit"].asInt();
    if (root.isMember("encoder_stop_bit") && root["encoder_stop_bit"].isInt()) encoder_stop_bit = root["encoder_stop_bit"].asInt();
    if (root.isMember("encoder_slave") && root["encoder_slave"].isInt()) encoder_slave = root["encoder_slave"].asInt();

    if (root.isMember("laser_scale_m") && root["laser_scale_m"].isNumeric()) laser_scale_m = root["laser_scale_m"].asFloat();
    if (root.isMember("laser1_tilt_deg") && root["laser1_tilt_deg"].isNumeric()) laser1_tilt_deg = root["laser1_tilt_deg"].asFloat();
    if (root.isMember("laser2_tilt_deg") && root["laser2_tilt_deg"].isNumeric()) laser2_tilt_deg = root["laser2_tilt_deg"].asFloat();

    if (root.isMember("encoder_k") && root["encoder_k"].isNumeric()) encoder_k = root["encoder_k"].asFloat();
    if (root.isMember("encoder_b") && root["encoder_b"].isNumeric()) encoder_b = root["encoder_b"].asFloat();

    if (root.isMember("loop_ms") && root["loop_ms"].isInt()) loop_ms = root["loop_ms"].asInt();

    return true;
}
