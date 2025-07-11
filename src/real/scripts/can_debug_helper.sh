#!/bin/bash

# CAN通信调试助手脚本
# 用于快速查看CAN接口状态和数据

echo "=== CAN通信调试助手 ==="
echo

# 检查CAN接口状态
echo "1. 检查CAN接口状态:"
ip link show can0 2>/dev/null || echo "  CAN接口 can0 未找到，请检查硬件连接"
echo

# 检查CAN接口配置
echo "2. CAN接口配置信息:"
if command -v canconfig &> /dev/null; then
    canconfig can0
else
    echo "  canconfig 工具未安装"
fi
echo

# 提供常用调试命令
echo "3. 常用CAN调试命令:"
echo "   启用CAN接口:    sudo ip link set can0 up type can bitrate 500000"
echo "   关闭CAN接口:    sudo ip link set can0 down"
echo "   监听CAN数据:    candump can0"
echo "   发送测试数据:   cansend can0 123#DEADBEEF"
echo

# 提供ROS调试命令
echo "4. ROS CAN调试命令:"
echo "   启动CAN调试:    roslaunch real can_debug_test.launch"
echo "   查看底盘数据:   rostopic echo /chassis_data"
echo "   查看编码器数据: rostopic echo /encoder_data"
echo "   查看话题列表:   rostopic list"
echo

# 检查是否有ROS节点运行
echo "5. 当前ROS节点状态:"
if pgrep -f "can_data_receiver_node" > /dev/null; then
    echo "  ✓ CAN数据接收节点正在运行"
else
    echo "  ✗ CAN数据接收节点未运行"
fi
echo

echo "=== 使用步骤建议 ==="
echo "1. 连接CAN转USB设备到计算机"
echo "2. 配置CAN接口: sudo ip link set can0 up type can bitrate 500000"
echo "3. 启动调试节点: roslaunch real can_debug_test.launch"
echo "4. 使用遥控器启动电机"
echo "5. 观察终端输出的CAN数据和速度计算结果"
echo "6. 调试完成后关闭: sudo ip link set can0 down"
echo

# 提供权限检查
echo "6. 权限检查:"
if groups $USER | grep -q "dialout"; then
    echo "  ✓ 用户在dialout组中，有串口权限"
else
    echo "  ✗ 用户不在dialout组中，可能需要添加权限:"
    echo "    sudo usermod -a -G dialout $USER"
    echo "    然后重新登录"
fi
