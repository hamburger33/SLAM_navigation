#include "real/wheel_odom_calculator.h"
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "wheel_odom_calculator_node");
    
    try {
        real::WheelOdomCalculator calculator;
        
        ROS_INFO("Wheel Odometry Calculator Node started");
        ros::spin();
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in Wheel Odometry Calculator: %s", e.what());
        return -1;
    }
    
    return 0;
}
