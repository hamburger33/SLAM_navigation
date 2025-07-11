#include "real/can_data_receiver.h"
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "can_data_receiver_node");
    
    try {
        real::CanDataReceiver receiver;
        receiver.start();
        
        ROS_INFO("CAN Data Receiver Node started");
        ros::spin();
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in CAN Data Receiver: %s", e.what());
        return -1;
    }
    
    return 0;
}
