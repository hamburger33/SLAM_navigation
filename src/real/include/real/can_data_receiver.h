#ifndef REAL_CAN_DATA_RECEIVER_H
#define REAL_CAN_DATA_RECEIVER_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include "real/chassis_data.h"

// 电机CAN ID定义 - 根据电机通讯协议
#define MOTOR_CAN_ID        0x1801E003  // 电机控制器CAN ID（包含左右电机编码器数据）
#define FAULT_CAN_ID        0x1801E002  // 故障信息帧CAN ID

// 编码器参数定义
#define ENCODER_RESOLUTION  4096        // 编码器分辨率：4096脉冲/圈
#define UPDATE_FREQUENCY    100.0       // 更新频率：100Hz (10ms更新一次)

// 根据用户提供的机器人物理参数
#define WHEEL_BASE          0.68        // 轮距：680mm = 0.68m
#define WHEEL_RADIUS        0.135       // 轮半径：270mm直径 = 135mm半径 = 0.135m
#define GEAR_RATIO          1.0         // 传动比：1:1 (电机直接驱动轮子)

// 数据格式参数 - 根据用户确认
#define ENCODER_BIG_ENDIAN  true        // 大端序
#define ENCODER_SIGNED      true        // 有符号32位整数
#define LEFT_MOTOR_DIRECTION  1         // 正值前进
#define RIGHT_MOTOR_DIRECTION 1         // 正值前进

// CAN通信调试开关 - 正式使用时设为false
#define CAN_DEBUG_PRINT     true        // 开启CAN调试打印

namespace real {

class CanDataReceiver {
public:
    CanDataReceiver();
    ~CanDataReceiver() = default;

    // 禁用拷贝构造和赋值
    CanDataReceiver(const CanDataReceiver&) = delete;
    CanDataReceiver& operator=(const CanDataReceiver&) = delete;

    // 启动CAN数据接收
    void start();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    // 发布者
    ros::Publisher chassis_data_pub_;
    ros::Publisher encoder_data_pub_;
    ros::Publisher fault_data_pub_;
    
    // 订阅者（如果有从其他节点接收CAN原始数据）
    ros::Subscriber raw_can_sub_;
    
    // 当前底盘数据
    Update_chassis current_chassis_data_;
    ChassisEncoderData current_encoder_data_;
    uint32_t current_fault_code_;
    
    // 用于速度计算的历史数据
    int32_t last_left_encoder_;
    int32_t last_right_encoder_;
    ros::Time last_update_time_;
    
    // 配置参数
    std::string can_interface_;
    std::string chassis_data_topic_;
    std::string encoder_data_topic_;
    std::string fault_data_topic_;
    int can_baudrate_;
    
    // CAN数据处理
    bool initializeCanInterface();
    void parseCanMessage(uint32_t can_id, const uint8_t* data, uint8_t len);
    void parseMotorEncoderData(const uint8_t* data, uint8_t len);
    void parseFaultData(const uint8_t* data, uint8_t len);
    void calculateChassisVelocity();
    void stop();
    
    // ROS回调函数
    void rawCanCallback(const std_msgs::String::ConstPtr& msg);
    
    // 发布数据
    void publishChassisData();
    void publishEncoderData();
    void publishFaultData();
    
    // 故障代码解析
    const char* getFaultDescription(uint32_t fault_code);
    
    // 参数加载
    void loadParameters();
    
    // 定时器回调
    ros::Timer data_timer_;
    void dataTimerCallback(const ros::TimerEvent& event);

    // 编码器数据重置（用于清零累积误差）
    void resetEncoderData();
    
    // 编码器数据有效性检查
    bool isEncoderDataValid(int32_t left_enc, int32_t right_enc);
};

} // namespace real

#endif // REAL_CAN_DATA_RECEIVER_H
