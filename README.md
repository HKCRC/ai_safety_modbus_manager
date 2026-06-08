# ai_safety_modbus_manager 编译步骤

## 1. 环境准备

- 操作系统：Linux
- 编译器：支持 C++17 的 `g++`
- 构建工具：`cmake`、`make`
- 依赖库：`Boost`、`jsoncpp`

安装示例（Ubuntu）：

```bash
sudo apt update
sudo apt install -y build-essential cmake libboost-all-dev libjsoncpp-dev
```

## 2. 代码目录要求

当前工程会在 CMake 中直接引用同级目录的以下项目：

- `../ai_safety_common`
- `../crane_log`

目录结构示例：

```bash
/root/ai_safety/
├── ai_safety_common
├── ai_safety_modbus_manager
└── crane_log
```

## 3. 配置并编译

进入工程目录：

```bash
cd /root/ai_safety/ai_safety_modbus_manager
```

首次编译或重新生成构建目录：

```bash
cmake -S . -B build
cmake --build build -j$(nproc)
```

## 4. 编译产物

本仓库当前只生成库，不生成可执行程序。

默认生成静态库：

```bash
build/libai_safety_modbus_manager.a
```

## 5. 可选：编译为动态库

如果需要生成动态库，可执行：

```bash
cmake -S . -B build -DMODBUS_MANAGER_BUILD_SHARED=ON
cmake --build build -j$(nproc)
```

## 6. 常见问题

- 如果提示找不到 `ai_safety_common` 或 `crane_log`，先检查它们是否位于当前工程同级目录。
- 如果提示找不到 `jsoncpp`，先安装 `libjsoncpp-dev`，或通过 `CMAKE_PREFIX_PATH` 指定安装路径。
- 如果只是增量编译，直接执行下面命令即可：

```bash
cmake --build build -j$(nproc)
```
