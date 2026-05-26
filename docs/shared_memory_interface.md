# ai_safety 共享内存侧接口文档

本文档定义 `ai_safety_modbus_manager` 与共享内存通讯的接口契约。

## 1. 信号接口定义

头文件引用：`ai_safety/include/shared_memory_bridge.h`
数据结构依赖：`ai_safety_common/shared_memory_types.hpp`

接口通过 4 个全局 `boost::signals2::signal` 暴露：

```cpp
// 1. 发送至共享内存：设备状态 (主动发布)
inline boost::signals2::signal<void(ai_safety_common::DeviceStatus)> SignalDeviceStatus;

// 2. 发送至共享内存：吊钩距离状态 (主动发布)
inline boost::signals2::signal<void(ai_safety_common::CraneState)> SignalCraneState;

// 3. 从共享内存读取：警报信息 (通过引用参数回传)
inline boost::signals2::signal<void(ai_safety_common::AlertMessage&)> SignalAlert;

// 4. 从共享内存读取：电源指令 (通过引用参数回传)
// 取值说明: 0 = None, 1 = PowerOn, 2 = PowerOff
inline boost::signals2::signal<void(std::uint8_t&)> SignalPowerButton;
```

## 2. 周期调用时序

`ai_safety` 内部在一个周期线程中按以下顺序调用信号（共享内存侧只需绑定对应的 slot 即可）：

1. 组装并发布状态：`SignalDeviceStatus(device_status)`
2. 组装并发布状态：`SignalCraneState(crane_state)`
3. 声明并读取警报指令：`SignalAlert(alert)`
4. 声明并读取电源指令：`SignalPowerButton(power_command)`

## 3. 对接说明

- **主动发布**的信号 (`SignalDeviceStatus`, `SignalCraneState`)：共享内存侧通过绑定的 slot 收到信号后，将其写入共享内存。
- **主动读取**的信号 (`SignalAlert`, `SignalPowerButton`)：共享内存侧通过绑定的 slot 收到信号后，将从共享内存读取到的最新值填充到对应的引用参数中返回。
