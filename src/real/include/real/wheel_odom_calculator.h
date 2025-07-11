#ifndef REAL_WHEEL_ODOM_CALCULATOR_H
#define REAL_WHEEL_ODOM_CALCULATOR_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <std_msgs/String.h>
#include "real/chassis_data.h"

// 使用与CAN数据接收器相同的底盘参数宏定义
#define WHEEL_BASE_ODOM         0.68        // 轮距：680mm = 0.68m
#define WHEEL_RADIUS_ODOM       0.135       // 轮半径：270mm直径 = 135mm半径 = 0.135m
#define ENCODER_RESOLUTION_ODOM 4096        // 编码器分辨率：4096脉冲/圈

namespace real {

class WheelOdomCalculator {
public:
    WheelOdomCalculator();
    ~WheelOdomCalculator() = default;

    // 禁用拷贝构造和赋值
    WheelOdomCalculator(const WheelOdomCalculator&) = delete;
    WheelOdomCalculator& operator=(const WheelOdomCalculator&) = delete;

private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    // 发布者
    ros::Publisher wheel_odom_pub_;
    
    // 订阅者
    ros::Subscriber encoder_data_sub_;
    ros::Subscriber chassis_data_sub_;
    
    // TF广播器
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    
    // 里程计状态
    double x_;              // X位置 (m)
    double y_;              // Y位置 (m)
    double theta_;          // 朝向角 (rad)
    double vx_;             // X方向速度 (m/s)
    double vy_;             // Y方向速度 (m/s)
    double vth_;            // 角速度 (rad/s)
    
    // 编码器状态
    int32_t last_left_encoder_;
    int32_t last_right_encoder_;
    ros::Time last_time_;
    bool first_encoder_data_;
    
    // 底盘参数
    ChassisParams chassis_params_;
    
    // 配置参数
    std::string base_frame_;
    std::string odom_frame_;
    std::string wheel_odom_topic_;
    bool publish_tf_;
    
    // 回调函数
    void encoderDataCallback(const std_msgs::String::ConstPtr& msg);
    void chassisDataCallback(const std_msgs::String::ConstPtr& msg);
    
    // 里程计计算
    void calculateOdometry(const ChassisEncoderData& encoder_data);
    void updateOdometryFromVelocity(const Update_chassis& chassis_data, double dt);
    
    // 发布里程计
    void publishWheelOdometry(const ros::Time& current_time);
    void publishTransform(const ros::Time& current_time);
    
    // 工具函数
    double normalizeAngle(double angle);
    void resetOdometry();
    
    // 参数加载
    void loadParameters();
    
    // 编码器计数转换
    double encoderTicksToMeters(int32_t ticks);
};

} // namespace real

#endif // REAL_WHEEL_ODOM_CALCULATOR_H
