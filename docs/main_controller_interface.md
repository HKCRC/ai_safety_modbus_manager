# 总控主程序接口文档

## 1. 文档目的

本文档面向整个 AI Safety 系统的总控主程序开发者，说明 `ai_safety_modbus_manager` 模块的生命周期控制接口、启动方式与配置格式。

## 2. 对外暴露接口

头文件：`ai_safety/include/manager_client.h`

```cpp
class ModbusManagerClient {
public:
    struct Status {
        bool ok = true;
        std::string message;
    };

    ModbusManagerClient();
    ~ModbusManagerClient();

    // 1. 加载配置
    Status loadConfig(const std::string& path);
    
    // 2. 初始化设备连接
    Status init();
    
    // 3. 启动后台运行线程
    Status start();
    
    // 4. 停止运行并释放资源
    Status stop();
};
```

## 3. 调用时序说明

调用顺序要求如下：

```cpp
ModbusManagerClient client;

// 1. 加载配置
if (auto status = client.loadConfig("ai_safety/config/modbus_config.json"); !status.ok) {
    // 处理错误: status.message
}

// 2. 初始化
if (auto status = client.init(); !status.ok) {
    // 处理错误: status.message (注：小车必须连接成功才能初始化成功)
}

// 3. 启动
if (auto status = client.start(); !status.ok) {
    // 处理错误: status.message
}

// 4. 退出前停止
client.stop();
```

## 4. 配置文件格式

配置文件为 JSON 格式，包含连接参数及标定参数。默认路径示例：`ai_safety/config/modbus_config.json`。

```json
{
    "crane_type": "flat",
    "x86_ip": "192.168.61.2",
    "trolley_ip": "192.168.61.36",
    "cab_bridge_ip": "192.168.61.67",
    "trolley_bridge_ip": "192.168.61.62",
    "trolley_port": 502,
    "trolley_slave": 1,
    "hook_dev": "/dev/ttyS0",
    "hook_baud": 9600,
    "hook_parity": "N",
    "hook_data_bit": 8,
    "hook_stop_bit": 1,
    "hook_slave": 3,
    "encoder_dev": "/dev/ttyS1",
    "encoder_baud": 9600,
    "encoder_parity": "E",
    "encoder_data_bit": 8,
    "encoder_stop_bit": 1,
    "encoder_slave": 1,
    "laser_scale_m": 0.001,
    "laser1_tilt_deg": 4.0,
    "laser2_tilt_deg": 12.0,
    "encoder_k": 1.0,
    "encoder_b": 0.0,
    "loop_ms": 1000
}
```

## 5. 数据交互说明

本模块的业务数据交互（如设备状态上报、告警指令下发）**不通过**上述生命周期函数进行。

数据流交互完全通过 `boost::signals2::signal` 实现，具体契约请参考：
[shared\_memory\_interface.md](shared_memory_interface.md)
