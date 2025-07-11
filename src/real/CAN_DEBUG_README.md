# CAN通信调试指南

## 系统架构说明

### 电机配置
- **四轮驱动**: 总共4个电机（左右各2个）
- **单个控制器**: 
  - 电机控制器: 控制所有电机 (CAN ID: 0x1801E003)
  - 故障信息帧: 提供系统故障状态 (CAN ID: 0x1801E002)

### 数据格式

#### 编码器数据帧 (0x1801E003)
每个CAN帧包含8字节数据：
- **Data0-3**: 左侧电机编码器位置 (32位有符号整数，大端序)
- **Data4-7**: 右侧电机编码器位置 (32位有符号整数，大端序)

#### 故障信息帧 (0x1801E002)
每个CAN帧包含8字节数据：
- **Data0-3**: 保留字段
- **Data4-7**: 故障代码 (32位无符号整数，大端序)

### 数据处理策略
- **左轮编码器** = 左侧电机编码器值 (Data0-3)
- **右轮编码器** = 右侧电机编码器值 (Data4-7)
- **故障监控** = 实时解析故障代码并发布

### 当前配置 (硬编码在头文件中)
- 轮距: 680mm (0.68m)
- 轮半径: 135mm (0.135m)
- 传动比: 1:1 (直连)
- 编码器分辨率: 4096脉冲/圈
- 字节序: 大端序
- 数据格式: 有符号32位整数

## 调试准备

### 1. 硬件连接
- 将CAN转USB设备连接到计算机
- 确保CAN总线与电机控制器正确连接
- 检查CANH和CANL线路连接正确

### 2. 软件环境
确保安装了CAN工具：
```bash
sudo apt-get install can-utils
```

### 3. 配置CAN接口
```bash
# 启用CAN接口 (波特率500kbps)
sudo ip link set can0 up type can bitrate 500000

# 检查CAN接口状态
ip link show can0
```

## 调试步骤

### 第1步：启动调试助手
```bash
# 查看调试助手信息
/home/yaya/simu_ws/xju-robot/src/real/scripts/can_debug_helper.sh
```

### 第2步：启动CAN调试节点
```bash
# 启动CAN通信调试节点
roslaunch real can_debug_test.launch
```

### 第3步：使用遥控器测试
1. 启动电机遥控器
2. 控制机器人运动
3. 观察终端输出的CAN数据

## 调试输出解释

### CAN原始数据
```
=== CAN Data Received ===
CAN ID: 0x1801E003, Length: 8 bytes
Raw Data: 0x00 0x00 0x12 0x34 0x00 0x00 0x56 0x78
>>> LEFT CONTROLLER Data (Controls 2 motors) <<<
```

### 编码器解析
```
Controller: LEFT
  Data0-3 (Motor1): 0x00 0x00 0x12 0x34 -> 4660 pulses
  Data4-7 (Motor2): 0x00 0x00 0x56 0x78 -> 22136 pulses
  Left Average Encoder: (4660 + 22136) / 2 = 13398
  Final Left Encoder: 13398
```

### 速度计算
```
--- Velocity Calculation ---
Time Delta: 0.0100 s
Encoder Values - Left: 4660, Right: 4672
Encoder Deltas - Left: 10, Right: 12 pulses
Wheel Angles - Left: 0.0153, Right: 0.0184 rad
Wheel Velocities - Left: 0.2069, Right: 0.2483 m/s
*** CHASSIS MOTION ***
  Linear Velocity (vx): 0.2276 m/s
  Angular Velocity (wz): 0.0609 rad/s
  Status: MOVING
```

## 参数验证

### 物理参数检查
当前配置的参数：
- 轮距 (wheel_base): 0.68m (680mm)
- 轮半径 (wheel_radius): 0.135m (270mm直径)
- 传动比 (gear_ratio): 1.0 (直连)
- 编码器分辨率: 4096脉冲/圈

### CAN ID验证
- 电机控制器: 0x1801E003 ✓ (包含左右电机编码器数据)
- 故障信息帧: 0x1801E002 ✓ (包含系统故障状态)

## 常见问题排除

### 1. 无法接收CAN数据
```bash
# 检查CAN接口
ip link show can0

# 监听原始CAN数据
candump can0

# 检查权限
groups $USER | grep dialout
```

### 2. 编码器数据异常
- 检查字节序设置 (当前: 大端序)
- 验证数据符号 (当前: 有符号)
- 确认电机方向 (当前: 正值前进)

### 3. 速度计算错误
- 验证物理参数 (轮距、轮半径)
- 检查传动比设置
- 确认编码器分辨率

## 关闭调试模式

调试完成后，修改代码中的调试开关：
```cpp
// 在 can_data_receiver.cpp 中
#define CAN_DEBUG_PRINT     false       // 关闭CAN调试打印
```

然后重新编译：
```bash
cd /home/yaya/simu_ws/xju-robot
catkin_make
```

## 数据记录

可以使用rosbag记录调试数据：
```bash
rosbag record -o can_debug_data /chassis_data /encoder_data
```

回放数据：
```bash
rosbag play can_debug_data_*.bag
```

### 故障代码表
| 代码 | 描述 | 代码 | 描述 |
|------|------|------|------|
| 0 | 正常 | 16 | 电机1缺相 |
| 1 | 驱动器1故障 | 17 | 电机2抱闸 |
| 2 | 驱动器2故障 | 18 | 电机1抱闸 |
| 3 | 驱动器3故障 | 19 | 电机2编码器故障 |
| 4 | 驱动器4故障 | 20 | 电机1编码器故障 |
| 5 | 过流 | 21 | 电机2过温 |
| 6 | 过压 | 22 | 电机1过温 |
| 7 | 欠压 | 23 | 电机2霍尔故障 |
| 8 | 过温 | 24 | 电机1霍尔故障 |
| 9-10 | 保留 | 25 | 电机2堵转 |
| 11 | 电机2超速 | 26 | 电机1堵转 |
| 12 | 电机1超速 | 27 | UART通讯故障 |
| 13 | 电机2过载 | 28 | RS485通讯故障 |
| 14 | 电机1过载 | 29 | CAN通讯故障 |
| 15 | 电机2缺相 | 30-31 | 摇杆/转把故障 |
