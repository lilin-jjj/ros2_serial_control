# ROS2 Turbojet Control System（涡轮喷气发动机控制系统）


## 功能特性

### 1. 信号生成器节点 (`signal_generator`)
支持**两种运行模式**：

#### 自动时间驱动模式
- 0-300秒：多阶梯阶跃信号(20个阶梯)
- 300-400秒：Chirp扫频信号
- 400-405秒：水平过渡信号
- 405-505秒：三段斜坡信号
- 505秒后：恒定值700

#### 命令驱动模式
通过订阅 `control_command` 话题接收控制指令，实时控制油门值：
- **start**：启动引擎并进入自动时间驱动序列
- **stop**：停止引擎，油门设为0
- **oill add** / **oil add**：每次增加油门 5%（增加 50 单位）
- **oill sub** / **oil sub**：每次减少油门 5%（减少 50 单位）
- **set <value>**：直接设置油门值（0-1000）
- **auto** / **resume**：从命令控制切回自动时间序列

### 2. 串口通信节点 (`serial_comm`)
#### 发送
- 将信号封装为ECU协议格式并通过串口发送
- Byte0：帧头(0xFF)
- Byte1：包含命令ID、开关状态和油门高位
- Byte2：油门低位
- Byte3：CRC校验码(反射CRC-8算法)

#### 接收与反馈处理（NEW）
- 后台线程实时接收ECU反馈数据
- 自动CRC验证
- 解析并打印ECU状态信息：
  - **引擎状态**：Off/Starting/Idle/Running/Stopping/Fault
  - **错误代码**：No Error/Over Temperature/Low Voltage/High Voltage/Sensor Error/Fuel Error 等
  - **发动机温度**：实时温度信息（单位°C）

## 目录结构

```
ros2_serial_control/
├── src/
│   └── turbojet_control/
│       ├── include/turbojet_control/     # 头文件
│       │   ├── signal_generator.hpp      # 信号生成器头文件
│       │   └── serial_comm.hpp          # 串口通信头文件
│       ├── src/                          # 源文件
│       │   ├── signal_generator.cpp      # 信号生成器实现
│       │   └── serial_comm.cpp          # 串口通信实现
│       ├── launch/                       # 启动文件
│       │   └── turbojet_control.launch.py # 系统启动配置
│       ├── params/                       # 参数文件
│       │   └── serial_params.yaml       # 默认串口参数
│       ├── CMakeLists.txt                # 构建配置
│       └── package.xml                   # 包信息
├── install/                              # 编译安装目录
└── README.md
```

## 系统架构

系统由两个ROS2节点组成，采用发布-订阅模式进行通信：

```
signal_generator → (throttle_signal) → serial_comm → (UART) → ECU
```

### signal_generator 节点
- **功能**: 按时间阶段生成复杂油门控制信号
- **发布话题**: `throttle_signal` (std_msgs/msg/UInt16)
- **定时器**: 20Hz (50ms间隔)生成信号
- **信号类型**:
  - 阶跃信号: 用于测试发动机对突变油门指令的瞬态响应能力
  - Chirp扫频信号: 频率渐变振荡信号，用于分析系统宽频域动态特性与稳定性边界
  - 水平过渡信号: 提供阶段间稳定期，减少测试阶段间相互干扰
  - 斜坡信号: 测试系统对连续线性变化输入的跟踪能力与动态响应速度
  - 恒定值信号: 提供稳态测试条件，评估长期运行稳定性与精度

### serial_comm 节点
- **功能**: 订阅油门信号，封装为ECU协议并通过串口发送
- **订阅话题**: `throttle_signal` (std_msgs/msg/UInt16)
- **串口通信**: 使用ROS2 serial_driver库
- **SW状态**: 根据时间自动切换发动机工作状态
  - 时间 < 1秒: SW=0 (不控制引擎)
  - 1-3秒: SW=2 (待机状态)
  - 3-560秒: SW=3 (运行状态)
  - 560秒后: SW=1 (停止状态)

## 依赖项

- ROS2 (Foxy或更新版本，推荐使用Humble)
- serial_driver库：用于串口通信

## 安装

1. 确保已安装ROS2环境(Humble推荐)
2. 安装serial_driver库及相关依赖：
   ```bash
   sudo apt-get install ros-humble-serial-driver
   sudo apt-get install ros-humble-asio-cmake-module
   ```
3. 将此包放入你的ROS2工作空间的src目录中
4. 编译：
   ```bash
   cd ros2_serial_control
   colcon build --packages-select turbojet_control
   ```

## 使用方法

1. 源化环境：
   ```bash
   source install/setup.bash
   ```

2. 运行节点：
   ```bash
   # 使用默认参数
   ros2 launch turbojet_control turbojet_control.launch.py
   
   # 或指定串口参数
   ros2 launch turbojet_control turbojet_control.launch.py port_name:=/dev/ttyUSB0 baud_rate:=9600
   ```

3. 或者单独运行节点：
   ```bash
   ros2 run turbojet_control signal_generator
   ros2 run turbojet_control serial_comm --ros-args -p port_name:=/dev/ttyUSB0 -p baud_rate:=9600
   ```

## ROS2接口

### 发布的话题
- `throttle_signal` (std_msgs/msg/UInt16) - 发送油门信号值(0-1000)

### 订阅的话题
- `throttle_signal` (std_msgs/msg/UInt16) - 接收油门信号值并封装为ECU协议
- `control_command` (std_msgs/msg/String) - **NEW** 接收控制指令（start/stop/oill add/oill sub/set/auto等）

### 参数
- `port_name` (string, 默认: "/dev/ttyUSB0") - 串口设备名称
- `baud_rate` (int, 默认: 9600) - 串口波特率

## 控制命令详解

通过ROS2 topic pub发送控制命令给信号生成器。命令格式为字符串，支持以下操作：

### 启动/停止控制
```bash
# 启动自动时间驱动序列（重置时间）
ros2 topic pub -1 /control_command std_msgs/msg/String "{data: 'start'}"

# 停止发动机，油门设为0
ros2 topic pub -1 /control_command std_msgs/msg/String "{data: 'stop'}"

# 从命令模式切换回自动时间序列（不重置时间）
ros2 topic pub -1 /control_command std_msgs/msg/String "{data: 'auto'}"
ros2 topic pub -1 /control_command std_msgs/msg/String "{data: 'resume'}"
```

### 油门增减控制
```bash
# 增加油门 5%（每次增加 50 单位）
ros2 topic pub -1 /control_command std_msgs/msg/String "{data: 'oill add'}"
ros2 topic pub -1 /control_command std_msgs/msg/String "{data: 'oil add'}"

# 减少油门 5%（每次减少 50 单位）
ros2 topic pub -1 /control_command std_msgs/msg/String "{data: 'oill sub'}"
ros2 topic pub -1 /control_command std_msgs/msg/String "{data: 'oil sub'}"
```

### 直接设置油门
```bash
# 直接设置油门值为500 (范围 0-1000)
ros2 topic pub -1 /control_command std_msgs/msg/String "{data: 'set 500'}"

# 设置油门为100
ros2 topic pub -1 /control_command std_msgs/msg/String "{data: 'set 100'}"
```

**说明**：
- 所有油门值自动剪裁到 [0, 1000] 范围
- `oill add` 和 `oil add` 增量为满刻度的 5%（1000 × 0.05 = 50 单位）
- `oill sub` 和 `oil sub` 减量相同
- 命令执行在下一个定时器周期生效（最多 50ms 延迟）

## ECU通信协议

### 发送格式（主机 → ECU）
4字节消息格式：
```
[Byte0] [Byte1] [Byte2] [Byte3]
   |       |       |       |
  0xFF    CMD+SW+TH_H  TH_L   CRC
```

- Byte0: 帧头 (固定值 0xFF)
- Byte1: 包含命令ID(4位)、开关状态(2位)和油门高位(2位)
- Byte2: 油门低位(8位)
- Byte3: CRC校验码(使用反射CRC-8算法，多项式0x8C)

具体位分配：
- CMD_ID: 1 (用于控制引擎状态和油门)
- SW状态值含义：
  - 0: 不控制引擎（PWM输入控制模式）
  - 1: 停止状态（超温不冷却）
  - 2: 待机状态（超温自动冷却）
  - 3: 运行状态
- TH: 油门值 (0-1000)
- CRC: 反射模式CRC-8校验，初始值0，校验范围为Byte1和Byte2

### 接收格式（ECU → 主机）反馈
4字节反馈格式：
```
[Byte0] [Byte1] [Byte2] [Byte3]
   |       |       |       |
  0xFF  Status+Error  Temp    CRC
```

- Byte0: 帧头 (0xFF)
- Byte1: 高4位为引擎状态，低4位为错误代码
  - **引擎状态**（高4位）：
    - 0x0: Off（关闭）
    - 0x1: Starting（启动中）
    - 0x2: Idle（空转）
    - 0x3: Running（运行中）
    - 0x4: Stopping（停止中）
    - 0x5: Fault（故障）
  - **错误代码**（低4位）：
    - 0x0: No Error（无错误）
    - 0x1: Over Temperature（超温）
    - 0x2: Low Voltage（低电压）
    - 0x3: High Voltage（高电压）
    - 0x4: Sensor Error（传感器错误）
    - 0x5: Fuel Error（燃油错误）
    - 0xF: Unspecified Error（未知错误）
- Byte2: 引擎温度 (原始值，转换公式：°C = 原始值 × 0.5 - 40)
- Byte3: CRC校验码

## 串口反馈日志示例

当ECU反馈数据时，系统会打印如下信息：

```
[INFO] ECU Feedback [FF 3A 50 xx]: Status=0x3, Error=0x0, Temp=80 (raw)
[INFO] [ECU Status] Engine: Running | Error: No Error | Temperature: 0.0°C
```

这表示：
- 引擎状态：运行中 (Status=0x3)
- 错误代码：无错误 (Error=0x0)
- 原始温度值：80 (转换后为 80*0.5 - 40 = 0°C)

## 故障排除

### 编译错误
如果遇到编译错误，请确保已正确安装所有依赖项：
```bash
sudo apt-get update
sudo apt-get install ros-humble-serial-driver ros-humble-asio-cmake-module
```

### 串口权限问题
如果遇到串口访问权限问题，请将用户添加到dialout组：
```bash
sudo usermod -a -G dialout $USER
```
然后重新登录系统。

### 串口连接问题
1. 确认串口设备存在：`ls /dev/ttyUSB*`
2. 确认串口设备权限：`ls -l /dev/ttyUSB*`
3. 使用正确的设备名称和波特率启动节点
