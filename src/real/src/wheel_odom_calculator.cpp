#include "real/wheel_odom_calculator.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sstream>

namespace real {

WheelOdomCalculator::WheelOdomCalculator()
    : nh_("/")
    , private_nh_("~")
    , x_(0.0)
    , y_(0.0)
    , theta_(0.0)
    , vx_(0.0)
    , vy_(0.0)
    , vth_(0.0)
    , last_left_encoder_(0)
    , last_right_encoder_(0)
    , first_encoder_data_(true) {
    
    // 加载参数
    loadParameters();
    
    // 初始化发布者
    wheel_odom_pub_ = nh_.advertise<nav_msgs::Odometry>(wheel_odom_topic_, 10);
    
    // 初始化订阅者
    encoder_data_sub_ = nh_.subscribe<std_msgs::String>(
        "/encoder_data", 10, &WheelOdomCalculator::encoderDataCallback, this);
    chassis_data_sub_ = nh_.subscribe<std_msgs::String>(
        "/chassis_data", 10, &WheelOdomCalculator::chassisDataCallback, this);
    
    last_time_ = ros::Time::now();
    
    ROS_INFO("Wheel Odometry Calculator initialized");
    ROS_INFO("Publishing wheel odometry on: %s", wheel_odom_topic_.c_str());
    ROS_INFO("Base frame: %s, Odom frame: %s", base_frame_.c_str(), odom_frame_.c_str());
}

void WheelOdomCalculator::loadParameters() {
    private_nh_.param<std::string>("base_frame", base_frame_, "base_link");
    private_nh_.param<std::string>("odom_frame", odom_frame_, "wheel_odom");
    private_nh_.param<std::string>("wheel_odom_topic", wheel_odom_topic_, "/wheel/odom");
    private_nh_.param<bool>("publish_tf", publish_tf_, true);
    
    // 底盘参数使用宏定义
    chassis_params_.wheel_base = WHEEL_BASE_ODOM;
    chassis_params_.wheel_radius = WHEEL_RADIUS_ODOM;
    chassis_params_.encoder_resolution = ENCODER_RESOLUTION_ODOM;
    
    ROS_INFO("Chassis params (macro-defined) - wheel_base: %.3f, wheel_radius: %.3f, encoder_resolution: %d",
             chassis_params_.wheel_base, chassis_params_.wheel_radius, chassis_params_.encoder_resolution);
}

void WheelOdomCalculator::encoderDataCallback(const std_msgs::String::ConstPtr& msg) {
    // 解析编码器数据
    ChassisEncoderData encoder_data;
    std::istringstream iss(msg->data);
    std::string token;
    
    // 简单的字符串解析（实际使用时可能需要更robust的解析）
    while (std::getline(iss, token, ',')) {
        if (token.find("left:") != std::string::npos) {
            encoder_data.left_encoder = std::stoi(token.substr(5));
        } else if (token.find("right:") != std::string::npos) {
            encoder_data.right_encoder = std::stoi(token.substr(6));
        } else if (token.find("timestamp:") != std::string::npos) {
            encoder_data.timestamp_us = std::stoull(token.substr(10));
        }
    }
    
    // 计算里程计
    calculateOdometry(encoder_data);
}

void WheelOdomCalculator::chassisDataCallback(const std_msgs::String::ConstPtr& msg) {
    // 解析底盘数据（可选，用于验证或补充编码器数据）
    Update_chassis chassis_data;
    std::istringstream iss(msg->data);
    std::string token;
    
    while (std::getline(iss, token, ',')) {
        if (token.find("linear_vel:") != std::string::npos) {
            chassis_data.linear_velocity = std::stof(token.substr(11));
        } else if (token.find("angular_vel:") != std::string::npos) {
            chassis_data.angular_velocity = std::stof(token.substr(12));
        }
    }
    
    // 可以使用速度数据进行补充计算或验证
    ros::Time current_time = ros::Time::now();
    double dt = (current_time - last_time_).toSec();
    if (dt > 0.001) { // 避免除零
        updateOdometryFromVelocity(chassis_data, dt);
    }
}

void WheelOdomCalculator::calculateOdometry(const ChassisEncoderData& encoder_data) {
    ros::Time current_time = ros::Time::now();
    
    if (first_encoder_data_) {
        last_left_encoder_ = encoder_data.left_encoder;
        last_right_encoder_ = encoder_data.right_encoder;
        last_time_ = current_time;
        first_encoder_data_ = false;
        return;
    }
    
    // 计算编码器变化
    int32_t delta_left = encoder_data.left_encoder - last_left_encoder_;
    int32_t delta_right = encoder_data.right_encoder - last_right_encoder_;
    
    // 转换为距离
    double delta_left_m = encoderTicksToMeters(delta_left);
    double delta_right_m = encoderTicksToMeters(delta_right);
    
    // 计算时间间隔
    double dt = (current_time - last_time_).toSec();
    
    if (dt > 0.001) { // 避免除零
        // 计算线速度和角速度
        double delta_distance = (delta_left_m + delta_right_m) / 2.0;
        double delta_theta = (delta_right_m - delta_left_m) / chassis_params_.wheel_base;
        
        vx_ = delta_distance / dt;
        vy_ = 0.0; // 差分驱动
        vth_ = delta_theta / dt;
        
        // 更新位置
        if (fabs(delta_theta) > 1e-6) {
            // 有转弯的情况
            double radius = delta_distance / delta_theta;
            x_ += radius * (sin(theta_ + delta_theta) - sin(theta_));
            y_ += -radius * (cos(theta_ + delta_theta) - cos(theta_));
        } else {
            // 直线运动
            x_ += delta_distance * cos(theta_);
            y_ += delta_distance * sin(theta_);
        }
        
        theta_ = normalizeAngle(theta_ + delta_theta);
        
        // 发布里程计
        publishWheelOdometry(current_time);
        
        if (publish_tf_) {
            publishTransform(current_time);
        }
    }
    
    // 更新上次状态
    last_left_encoder_ = encoder_data.left_encoder;
    last_right_encoder_ = encoder_data.right_encoder;
    last_time_ = current_time;
}

void WheelOdomCalculator::updateOdometryFromVelocity(const Update_chassis& chassis_data, double dt) {
    // 使用速度数据更新里程计（可选方法）
    double delta_x = chassis_data.linear_velocity * dt * cos(theta_);
    double delta_y = chassis_data.linear_velocity * dt * sin(theta_);
    double delta_theta = chassis_data.angular_velocity * dt;
    
    x_ += delta_x;
    y_ += delta_y;
    theta_ = normalizeAngle(theta_ + delta_theta);
    
    vx_ = chassis_data.linear_velocity;
    vy_ = 0.0; // 差分驱动，侧向速度为0
    vth_ = chassis_data.angular_velocity;
}

void WheelOdomCalculator::publishWheelOdometry(const ros::Time& current_time) {
    nav_msgs::Odometry odom_msg;
    
    // 设置header
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;
    
    // 设置位置
    odom_msg.pose.pose.position.x = x_;
    odom_msg.pose.pose.position.y = y_;
    odom_msg.pose.pose.position.z = 0.0;
    
    // 设置姿态
    tf2::Quaternion quat;
    quat.setRPY(0, 0, theta_);
    odom_msg.pose.pose.orientation = tf2::toMsg(quat);
    
    // 设置速度
    odom_msg.twist.twist.linear.x = vx_;
    odom_msg.twist.twist.linear.y = vy_;
    odom_msg.twist.twist.angular.z = vth_;
    
    // 设置协方差矩阵（简化处理）
    for (int i = 0; i < 36; i++) {
        odom_msg.pose.covariance[i] = 0.0;
        odom_msg.twist.covariance[i] = 0.0;
    }
    
    // 设置对角线元素
    odom_msg.pose.covariance[0] = 0.1;   // x
    odom_msg.pose.covariance[7] = 0.1;   // y
    odom_msg.pose.covariance[35] = 0.1;  // yaw
    
    odom_msg.twist.covariance[0] = 0.1;  // vx
    odom_msg.twist.covariance[7] = 0.1;  // vy
    odom_msg.twist.covariance[35] = 0.1; // vyaw
    
    wheel_odom_pub_.publish(odom_msg);
}

void WheelOdomCalculator::publishTransform(const ros::Time& current_time) {
    geometry_msgs::TransformStamped transform;
    
    transform.header.stamp = current_time;
    transform.header.frame_id = odom_frame_;
    transform.child_frame_id = base_frame_;
    
    transform.transform.translation.x = x_;
    transform.transform.translation.y = y_;
    transform.transform.translation.z = 0.0;
    
    tf2::Quaternion quat;
    quat.setRPY(0, 0, theta_);
    transform.transform.rotation = tf2::toMsg(quat);
    
    tf_broadcaster_.sendTransform(transform);
}

double WheelOdomCalculator::normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

void WheelOdomCalculator::resetOdometry() {
    x_ = 0.0;
    y_ = 0.0;
    theta_ = 0.0;
    vx_ = 0.0;
    vy_ = 0.0;
    vth_ = 0.0;
    ROS_INFO("Odometry reset to origin");
}

double WheelOdomCalculator::encoderTicksToMeters(int32_t ticks) {
    // 编码器计数转换为距离（米）
    double distance_per_tick = (2.0 * M_PI * chassis_params_.wheel_radius) / chassis_params_.encoder_resolution;
    return ticks * distance_per_tick;
}

} // namespace real
