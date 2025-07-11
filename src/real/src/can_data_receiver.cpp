#include "real/can_data_receiver.h"
#include <ros/ros.h>
#include <cstring>
#include <cmath>
#include <climits>  // for INT32_MAX, INT32_MIN

namespace real {

CanDataReceiver::CanDataReceiver() 
    : nh_("/")
    , private_nh_("~")
    , last_left_encoder_(0)
    , last_right_encoder_(0)
    , last_update_time_(ros::Time::now()) {
    
    // 加载参数
    loadParameters();
    
    // 初始化发布者
    chassis_data_pub_ = nh_.advertise<std_msgs::String>(chassis_data_topic_, 10);
    encoder_data_pub_ = nh_.advertise<std_msgs::String>(encoder_data_topic_, 10);
    fault_data_pub_ = nh_.advertise<std_msgs::String>(fault_data_topic_, 10);
    
    // 初始化订阅者（如果需要从其他节点接收CAN数据）
    raw_can_sub_ = nh_.subscribe<std_msgs::String>(
        "/raw_can_data", 10, &CanDataReceiver::rawCanCallback, this);
    
    // 初始化定时器（模拟CAN数据接收，实际使用时替换为真实CAN接口）
    data_timer_ = nh_.createTimer(ros::Duration(0.01), // 100Hz
                                 &CanDataReceiver::dataTimerCallback, this);
    
    // 初始化CAN接口
    if (!initializeCanInterface()) {
        ROS_ERROR("Failed to initialize CAN interface");
    }
    
    // 初始化底盘数据
    current_chassis_data_.status = ChassisStatus::STOPPED;
    current_chassis_data_.linear_velocity = 0.0;
    current_chassis_data_.angular_velocity = 0.0;
    current_fault_code_ = 0;  // 初始化为正常状态
    
    ROS_INFO("CAN Data Receiver initialized with parameters:");
    ROS_INFO("  - Wheel base: %.3f m", WHEEL_BASE);
    ROS_INFO("  - Wheel radius: %.3f m", WHEEL_RADIUS);
    ROS_INFO("  - Gear ratio: %.1f:1", GEAR_RATIO);
    ROS_INFO("  - Encoder resolution: %d pulses/rev", ENCODER_RESOLUTION);
}

void CanDataReceiver::loadParameters() {
    private_nh_.param<std::string>("can_interface", can_interface_, "can0");
    private_nh_.param<std::string>("chassis_data_topic", chassis_data_topic_, "/chassis_data");
    private_nh_.param<std::string>("encoder_data_topic", encoder_data_topic_, "/encoder_data");
    private_nh_.param<std::string>("fault_data_topic", fault_data_topic_, "/fault_data");
    private_nh_.param<int>("can_baudrate", can_baudrate_, 500000);
}

bool CanDataReceiver::initializeCanInterface() {
    // TODO: 实现真实的CAN接口初始化
    // 这里可以使用SocketCAN或其他CAN库
    ROS_INFO("Initializing CAN interface: %s at %d baud", 
             can_interface_.c_str(), can_baudrate_);
    return true;
}

void CanDataReceiver::start() {
    ROS_INFO("Starting CAN data reception...");
    // TODO: 启动CAN数据接收线程
    // 实际实现中，这里会启动一个线程来持续接收CAN数据
}

void CanDataReceiver::stop() {
    ROS_INFO("Stopping CAN data reception...");
    // TODO: 停止CAN数据接收线程
}

void CanDataReceiver::parseCanMessage(uint32_t id, const uint8_t* data, uint8_t len) {
    // CAN通信调试打印 - 接收到的原始数据
    if (CAN_DEBUG_PRINT) {
        printf("\n=== CAN Data Received ===\n");
        printf("CAN ID: 0x%08X, Length: %d bytes\n", id, len);
        printf("Raw Data: ");
        for (int i = 0; i < len; i++) {
            printf("0x%02X ", data[i]);
        }
        printf("\n");
    }
    
    // 根据电机通讯协议解析CAN消息
    switch (id) {
        case MOTOR_CAN_ID:   // 0x1801E003
            if (CAN_DEBUG_PRINT) {
                printf(">>> MOTOR CONTROLLER Data (Left+Right motors) <<<\n");
            }
            parseMotorEncoderData(data, len);
            break;
            
        case FAULT_CAN_ID:   // 0x1801E002
            if (CAN_DEBUG_PRINT) {
                printf(">>> FAULT INFO Data <<<\n");
            }
            parseFaultData(data, len);
            break;
            
        default:
            if (CAN_DEBUG_PRINT) {
                printf(">>> UNKNOWN CAN ID: 0x%X <<<\n", id);
            }
            ROS_DEBUG("Received unknown CAN ID: 0x%X", id);
            break;
    }
}

void CanDataReceiver::parseMotorEncoderData(const uint8_t* data, uint8_t len) {
    if (len < 8) {
        ROS_WARN("Insufficient CAN data length: %d", len);
        return;
    }
    
    // 根据电机通讯协议解析数据
    // Data0-3: 左侧电机编码器位置 (32位有符号整数，大端序)
    // Data4-7: 右侧电机编码器位置 (32位有符号整数，大端序)
    
    int32_t left_motor_encoder = 0;
    int32_t right_motor_encoder = 0;
    
    // 解析左侧电机编码器位置 (Data0-3) - 大端序 (高字节在前)
    left_motor_encoder = (int32_t(data[0]) << 24) | 
                        (int32_t(data[1]) << 16) | 
                        (int32_t(data[2]) << 8)  | 
                        int32_t(data[3]);
    
    // 解析右侧电机编码器位置 (Data4-7) - 大端序 (高字节在前)
    right_motor_encoder = (int32_t(data[4]) << 24) | 
                         (int32_t(data[5]) << 16) | 
                         (int32_t(data[6]) << 8)  | 
                         int32_t(data[7]);
    
    // CAN通信调试打印 - 编码器解析详情
    if (CAN_DEBUG_PRINT) {
        printf("Motor Encoder Data:\n");
        printf("  Data0-3 (Left Motor): 0x%02X 0x%02X 0x%02X 0x%02X -> %d pulses\n", 
               data[0], data[1], data[2], data[3], left_motor_encoder);
        printf("  Data4-7 (Right Motor): 0x%02X 0x%02X 0x%02X 0x%02X -> %d pulses\n", 
               data[4], data[5], data[6], data[7], right_motor_encoder);
    }
    
    // 直接使用左右电机的编码器值（应用方向修正）
    int32_t new_left_encoder = left_motor_encoder * LEFT_MOTOR_DIRECTION;
    int32_t new_right_encoder = right_motor_encoder * RIGHT_MOTOR_DIRECTION;
    
    // 检查编码器数据有效性
    if (!isEncoderDataValid(new_left_encoder, new_right_encoder)) {
        ROS_WARN("Invalid encoder data detected, resetting encoder baseline");
        // 如果数据接近溢出，重置基准值
        current_encoder_data_.left_encoder = new_left_encoder;
        current_encoder_data_.right_encoder = new_right_encoder;
        resetEncoderData();
        return;
    }
    
    current_encoder_data_.left_encoder = new_left_encoder;
    current_encoder_data_.right_encoder = new_right_encoder;
    
    if (CAN_DEBUG_PRINT) {
        printf("  Final Left Encoder: %d\n", current_encoder_data_.left_encoder);
        printf("  Final Right Encoder: %d\n", current_encoder_data_.right_encoder);
    }
    
    current_encoder_data_.timestamp_us = ros::Time::now().toNSec() / 1000;
    
    // 计算底盘速度和角速度
    calculateChassisVelocity();
}

void CanDataReceiver::calculateChassisVelocity() {
    ros::Time current_time = ros::Time::now();
    double dt = (current_time - last_update_time_).toSec();
    
    // 避免除零和过小的时间间隔
    if (dt < 0.001) {
        return;
    }
    
    // 计算编码器变化量并进行溢出保护
    int32_t left_delta = current_encoder_data_.left_encoder - last_left_encoder_;
    int32_t right_delta = current_encoder_data_.right_encoder - last_right_encoder_;
    
    // 检测编码器溢出（处理32位整数回绕）
    const int32_t OVERFLOW_THRESHOLD = INT32_MAX / 2;  // 约10亿
    
    if (abs(left_delta) > OVERFLOW_THRESHOLD) {
        if (left_delta > 0) {
            // 正向溢出：当前值很大，上次值很小（负数）
            left_delta = left_delta - (INT32_MAX - INT32_MIN + 1);
        } else {
            // 反向溢出：当前值很小（负数），上次值很大
            left_delta = left_delta + (INT32_MAX - INT32_MIN + 1);
        }
        ROS_WARN("Left encoder overflow detected, corrected delta: %d", left_delta);
    }
    
    if (abs(right_delta) > OVERFLOW_THRESHOLD) {
        if (right_delta > 0) {
            right_delta = right_delta - (INT32_MAX - INT32_MIN + 1);
        } else {
            right_delta = right_delta + (INT32_MAX - INT32_MIN + 1);
        }
        ROS_WARN("Right encoder overflow detected, corrected delta: %d", right_delta);
    }
    
    // 异常变化检测（可能的数据错误）
    const double MAX_REASONABLE_DISTANCE = 10.0; // 10米每次更新(在100Hz下相当于1000m/s)
    double left_distance_abs = abs((double)left_delta / ENCODER_RESOLUTION * (2.0 * M_PI * WHEEL_RADIUS));
    double right_distance_abs = abs((double)right_delta / ENCODER_RESOLUTION * (2.0 * M_PI * WHEEL_RADIUS));
    
    if (left_distance_abs > MAX_REASONABLE_DISTANCE || right_distance_abs > MAX_REASONABLE_DISTANCE) {
        ROS_WARN("Abnormal encoder change detected - Left: %.2fm, Right: %.2fm, ignoring this update", 
                 left_distance_abs, right_distance_abs);
        return;
    }
    
    // 将编码器脉冲转换为轮子移动距离 (m)
    // 编码器值已是"圈数×4096"，直接除以4096得到圈数，再乘以轮子周长
    double left_wheel_distance = (double)left_delta / ENCODER_RESOLUTION * (2.0 * M_PI * WHEEL_RADIUS);
    double right_wheel_distance = (double)right_delta / ENCODER_RESOLUTION * (2.0 * M_PI * WHEEL_RADIUS);
    
    // 计算轮子线速度 (m/s)
    double left_wheel_velocity = left_wheel_distance / dt;
    double right_wheel_velocity = right_wheel_distance / dt;
    
    // 计算底盘线速度和角速度 (差分驱动运动学)
    current_chassis_data_.linear_velocity = (left_wheel_velocity + right_wheel_velocity) / 2.0;
    current_chassis_data_.angular_velocity = (right_wheel_velocity - left_wheel_velocity) / WHEEL_BASE;
    
    // 更新状态
    double speed_threshold = 0.01; // m/s
    if (fabs(current_chassis_data_.linear_velocity) > speed_threshold || 
        fabs(current_chassis_data_.angular_velocity) > speed_threshold) {
        current_chassis_data_.status = ChassisStatus::MOVING;
    } else {
        current_chassis_data_.status = ChassisStatus::STOPPED;
    }
    
    // CAN通信调试打印 - 速度计算详情
    if (CAN_DEBUG_PRINT) {
        printf("\n--- Velocity Calculation ---\n");
        printf("Time Delta: %.4f s\n", dt);
        printf("Encoder Values - Left: %d, Right: %d\n", 
               current_encoder_data_.left_encoder, current_encoder_data_.right_encoder);
        printf("Encoder Deltas - Left: %d, Right: %d pulses\n", left_delta, right_delta);
        printf("Wheel Distances - Left: %.4f, Right: %.4f m\n", left_wheel_distance, right_wheel_distance);
        printf("Wheel Velocities - Left: %.4f, Right: %.4f m/s\n", left_wheel_velocity, right_wheel_velocity);
        printf("*** CHASSIS MOTION ***\n");
        printf("  Linear Velocity: %.4f m/s\n", current_chassis_data_.linear_velocity);
        printf("  Angular Velocity: %.4f rad/s\n", current_chassis_data_.angular_velocity);
        printf("  Status: %s\n", (current_chassis_data_.status == ChassisStatus::MOVING) ? "MOVING" : "STOPPED");
        printf("=========================\n\n");
    }
    
    // 更新历史数据
    last_left_encoder_ = current_encoder_data_.left_encoder;
    last_right_encoder_ = current_encoder_data_.right_encoder;
    last_update_time_ = current_time;
    
    ROS_DEBUG("Velocity - Linear: %.3f m/s, Angular: %.3f rad/s", 
              current_chassis_data_.linear_velocity, current_chassis_data_.angular_velocity);
}

void CanDataReceiver::dataTimerCallback(const ros::TimerEvent& event) {
    // 模拟数据接收回调（实际使用时用真实CAN数据替换）
    static int sim_counter = 0;
    sim_counter++;
    
    // 模拟编码器数据变化 - 模拟机器人缓慢前进并轻微转向
    double time_sec = ros::Time::now().toSec();
    int32_t sim_left_enc = (int32_t)(100.0 * time_sec); // 模拟左轮编码器
    int32_t sim_right_enc = (int32_t)(102.0 * time_sec); // 模拟右轮稍快，产生转向
    
    current_encoder_data_.left_encoder = sim_left_enc;
    current_encoder_data_.right_encoder = sim_right_enc;
    current_encoder_data_.timestamp_us = ros::Time::now().toNSec() / 1000;
    
    // 计算速度
    calculateChassisVelocity();
    
    // 发布数据
    publishChassisData();
    publishEncoderData();
    
    // 模拟故障数据（可选）
    static int fault_sim_counter = 0;
    fault_sim_counter++;
    if (fault_sim_counter % 1000 == 0) { // 每10秒发布一次故障状态
        publishFaultData();
    }
}

void CanDataReceiver::rawCanCallback(const std_msgs::String::ConstPtr& msg) {
    // TODO: 如果有其他节点发送原始CAN数据，在这里处理
    ROS_DEBUG("Received raw CAN data: %s", msg->data.c_str());
}

void CanDataReceiver::publishChassisData() {
    std_msgs::String msg;
    char buffer[256];
    snprintf(buffer, sizeof(buffer), 
             "status:%d,linear_vel:%.4f,angular_vel:%.4f,timestamp:%lu",
             static_cast<int>(current_chassis_data_.status),
             current_chassis_data_.linear_velocity,
             current_chassis_data_.angular_velocity,
             current_encoder_data_.timestamp_us);
    msg.data = buffer;
    
    chassis_data_pub_.publish(msg);
}

void CanDataReceiver::publishEncoderData() {
    std_msgs::String msg;
    char buffer[256];
    snprintf(buffer, sizeof(buffer), 
             "left:%d,right:%d,timestamp:%lu",
             current_encoder_data_.left_encoder,
             current_encoder_data_.right_encoder,
             current_encoder_data_.timestamp_us);
    msg.data = buffer;
    
    encoder_data_pub_.publish(msg);
}

void CanDataReceiver::parseFaultData(const uint8_t* data, uint8_t len) {
    if (len < 8) {
        ROS_WARN("Insufficient fault data length: %d", len);
        return;
    }
    
    // 解析故障信息 (Data4-7) - uint32类型，大端序
    uint32_t fault_code = (uint32_t(data[4]) << 24) | 
                         (uint32_t(data[5]) << 16) | 
                         (uint32_t(data[6]) << 8)  | 
                         uint32_t(data[7]);
    
    // CAN通信调试打印 - 故障信息详情
    if (CAN_DEBUG_PRINT) {
        printf("Fault Data:\n");
        printf("  Data4-7 (Fault Code): 0x%02X 0x%02X 0x%02X 0x%02X -> %u\n", 
               data[4], data[5], data[6], data[7], fault_code);
        printf("  Fault Description: %s\n", getFaultDescription(fault_code));
    }
    
    // 更新故障代码
    uint32_t previous_fault = current_fault_code_;
    current_fault_code_ = fault_code;
    
    // 如果故障状态发生变化，立即发布
    if (previous_fault != current_fault_code_) {
        if (current_fault_code_ == 0) {
            ROS_INFO("Fault cleared: System normal");
        } else {
            ROS_WARN("Fault detected: Code %u - %s", current_fault_code_, getFaultDescription(current_fault_code_));
        }
        publishFaultData();
    }
}

void CanDataReceiver::publishFaultData() {
    std_msgs::String msg;
    char buffer[256];
    snprintf(buffer, sizeof(buffer), 
             "fault_code:%u,description:%s,timestamp:%lu",
             current_fault_code_,
             getFaultDescription(current_fault_code_),
             ros::Time::now().toNSec() / 1000);
    msg.data = buffer;
    
    fault_data_pub_.publish(msg);
}

const char* CanDataReceiver::getFaultDescription(uint32_t fault_code) {
    switch (fault_code) {
        case 0:  return "正常";
        case 1:  return "驱动器1故障";
        case 2:  return "驱动器2故障";
        case 3:  return "驱动器3故障";
        case 4:  return "驱动器4故障";
        case 5:  return "过流";
        case 6:  return "过压";
        case 7:  return "欠压";
        case 8:  return "过温";
        case 9:  return "保留故障9";
        case 10: return "保留故障10";
        case 11: return "电机2超速";
        case 12: return "电机1超速";
        case 13: return "电机2过载";
        case 14: return "电机1过载";
        case 15: return "电机2缺相";
        case 16: return "电机1缺相";
        case 17: return "电机2抱闸";
        case 18: return "电机1抱闸";
        case 19: return "电机2编码器故障";
        case 20: return "电机1编码器故障";
        case 21: return "电机2过温";
        case 22: return "电机1过温";
        case 23: return "电机2霍尔故障";
        case 24: return "电机1霍尔故障";
        case 25: return "电机2堵转";
        case 26: return "电机1堵转";
        case 27: return "UART通讯故障";
        case 28: return "RS485通讯故障";
        case 29: return "CAN通讯故障";
        case 30: return "摇杆故障";
        case 31: return "转把故障";
        default: return "未知故障";
    }
}

void CanDataReceiver::resetEncoderData() {
    last_left_encoder_ = current_encoder_data_.left_encoder;
    last_right_encoder_ = current_encoder_data_.right_encoder;
    last_update_time_ = ros::Time::now();
    ROS_INFO("Encoder data reset to prevent accumulation errors");
}

bool CanDataReceiver::isEncoderDataValid(int32_t left_enc, int32_t right_enc) {
    // 检查编码器数据是否在合理范围内
    const int32_t MAX_ENCODER_VALUE = INT32_MAX - 1000000;  // 留出安全余量
    const int32_t MIN_ENCODER_VALUE = INT32_MIN + 1000000;
    
    if (left_enc > MAX_ENCODER_VALUE || left_enc < MIN_ENCODER_VALUE ||
        right_enc > MAX_ENCODER_VALUE || right_enc < MIN_ENCODER_VALUE) {
        ROS_WARN("Encoder values approaching overflow limits: Left=%d, Right=%d", left_enc, right_enc);
        return false;
    }
    
    return true;
}

} // namespace real
