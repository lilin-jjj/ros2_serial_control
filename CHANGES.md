# 代码更新说明（2025-11-12）

## 新增功能概述

### 1. 命令驱动油门控制（signal_generator）
将原有的**纯时间驱动**信号生成改为**命令可控**模式。

#### 修改的文件
- `include/turbojet_control/signal_generator.hpp`
- `src/turbojet_control/src/signal_generator.cpp`

#### 新增功能
- **订阅 `control_command` 话题**：接收 `std_msgs::msg::String` 类型的命令
- **命令种类**：
  - `start`：进入自动时间驱动序列（重置时间）
  - `stop`：停止发动机（油门 = 0）
  - `oill add` / `oil add`：增加油门 5%（+50 单位）
  - `oill sub` / `oil sub`：减少油门 5%（-50 单位）
  - `set <value>`：直接设置油门（0-1000）
  - `auto` / `resume`：切回自动时间序列（不重置时间）

#### 实现细节
- 新增 `command_override_` 标志：为真时发布命令设置的值，否则按自动序列发布
- 新增 `engine_started_` 标志：引擎是否启动
- 新增 `current_throttle_` 变量：存储命令设置的油门值
- 使用 `std::mutex` 保护共享状态（线程安全）
- `timer_callback()` 改为根据标志选择发布源（命令或自动序列）

### 2. ECU 反馈接收与解析（serial_comm）
新增**后台线程**接收并解析 ECU 发送的反馈信号。

#### 修改的文件
- `include/turbojet_control/serial_comm.hpp`
- `src/turbojet_control/src/serial_comm.cpp`

#### 新增功能
- **后台线程**：`receive_feedback_thread()` 持续监听串口反馈
- **反馈解析**：`parse_ecu_feedback()` 解析和打印 ECU 状态
- **CRC 验证**：自动验证接收到的反馈数据完整性
- **日志输出**：打印引擎状态、错误代码、温度等信息

#### 反馈信息格式（假设）
```
[Byte0] [Byte1] [Byte2] [Byte3]
  0xFF   Status+Error   Temp    CRC

Byte1 高 4 位：引擎状态（Off/Starting/Idle/Running/Stopping/Fault）
Byte1 低 4 位：错误代码（No Error/Over Temperature/Low Voltage 等）
Byte2：温度原始值（转换：°C = raw × 0.5 - 40）
Byte3：CRC-8 校验码
```

#### 实现细节
- 线程在构造函数中启动，在析构时停止
- 缓冲区管理：最大 1024 字节，溢出时自动清空
- 错误处理：CRC 校验失败时记录警告日志
- 状态转换：将数字代码转换为可读字符串（Running/Fault 等）

### 3. README 更新
- 更新功能描述（新增命令驱动模式、反馈接收）
- 添加详细的控制命令示例
- 添加 ECU 反馈格式和日志示例

## 使用示例

### 1. 启动节点
```bash
ros2 launch turbojet_control turbojet_control.launch.py
```

### 2. 发送命令（在另一个终端）
```bash
# 启动引擎
ros2 topic pub -1 /control_command std_msgs/msg/String "{data: 'start'}"

# 增加油门
ros2 topic pub -1 /control_command std_msgs/msg/String "{data: 'oill add'}"

# 设置油门为 600
ros2 topic pub -1 /control_command std_msgs/msg/String "{data: 'set 600'}"

# 停止引擎
ros2 topic pub -1 /control_command std_msgs/msg/String "{data: 'stop'}"

# 切回自动时间序列
ros2 topic pub -1 /control_command std_msgs/msg/String "{data: 'auto'}"
```

### 3. 观察输出
- 油门信号会在 `throttle_signal` 话题上发布
- `serial_comm` 节点会打印发送的命令和接收的反馈

## 编译状态
✓ 编译成功（colcon build --packages-select turbojet_control）
✓ 无编译错误或致命警告

## 向后兼容性
- 所有原有的时间驱动信号仍然保留
- 启动后默认不发送（`engine_started_` = false）
- 发送命令 `start` 后进入自动时间驱动序列
- 无需修改依赖或其他模块

## 后续建议
1. 根据实际 ECU 协议调整 `parse_ecu_feedback()` 的解析逻辑
2. 将 5% 增量提取为 ROS2 参数供在线调整
3. 添加命令状态反馈 topic（发布当前模式、油门值等）
4. 实现命令历史记录和日志功能
5. 添加单元测试和集成测试

---

# 代码更新说明（2025-11-28）

## 修复 SW 状态计算问题

### 问题描述
原代码中 SW 状态基于**节点启动时间**计算，导致节点运行超过 560 秒后，SW 状态会一直为 1（停止状态），即使发送 `set 600` 命令也无法正常控制引擎。

### 修复内容

#### 1. 添加引擎状态管理
- **新增成员变量**：
  - `engine_started_`: 标记引擎是否启动
  - `engine_start_time_`: 记录引擎启动时间
  - `state_mutex_`: 保护引擎状态的互斥锁

#### 2. 订阅 control_command 话题
`serial_comm` 节点现在也订阅 `control_command` 话题，监听以下命令：
- **`start`**: 启动引擎，重置启动时间
- **`stop`**: 停止引擎，SW 状态固定为 1
- **`auto`/`resume`**: 恢复自动模式（不重置时间）

#### 3. 修正 SW 状态计算逻辑
新增 `get_sw_state()` 方法：
- 如果引擎未启动：返回 SW = 1（停止状态）
- 如果引擎已启动：基于**引擎运行时间**计算 SW 状态
  - 0-1秒: SW = 0（不控制引擎）
  - 1-3秒: SW = 2（待机状态）
  - 3-560秒: SW = 3（运行状态）
  - >560秒: SW = 1（停止状态）

### 修改的文件
- `include/turbojet_control/serial_comm.hpp`
- `src/turbojet_control/src/serial_comm.cpp`

### 使用方法

#### 正确的命令序列
```bash
# 1. 先启动引擎（重置时间）
ros2 topic pub -1 /control_command std_msgs/msg/String "{data: 'start'}"

# 2. 等待 3 秒后（进入运行状态 SW=3），设置油门
ros2 topic pub -1 /control_command std_msgs/msg/String "{data: 'set 600'}"

# 3. 停止引擎
ros2 topic pub -1 /control_command std_msgs/msg/String "{data: 'stop'}"
```

### 预期行为
- 发送 `start` 后，SW 状态会按时间序列变化：0 → 2 → 3
- 在 SW=3 期间发送 `set 600`，串口消息中的 SW 字段应为 3（0b11）
- 发送 `stop` 后，SW 状态固定为 1

### 编译状态
✓ 编译成功（colcon build --packages-select turbojet_control）
✓ 仅有未使用参数警告（不影响功能）

### 验证方法
```bash
# 查看发送的串口消息（应该看到 SW 状态变化）
ros2 run turbojet_control serial_comm --ros-args --log-level debug
```
