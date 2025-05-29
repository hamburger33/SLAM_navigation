#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <can_msgs/CanFrame.h>
#include <dynamic_reconfigure/server.h>
#include "navigation/CanConfig.h"  // 动态参数配置头文件
#include "Cmd_chassis.h"

class NavigationNode {
private:
    ros::NodeHandle nh_;
    ros::Publisher can_pub_;
    ros::Subscriber cmd_vel_sub_;
    dynamic_reconfigure::Server<navigation::CanConfig> param_server_;  // 动态参数服务器

    // 可配置参数
    uint32_t can_id_ = 0x200;
    int can_dlc_ = 8;
    float max_linear_speed_ = 1.0;
    float max_angular_speed_ = 1.0;

public:
    NavigationNode() : nh_("~") {
        // 初始化参数
        loadParameters();

        // 设置动态参数回调
        param_server_.setCallback(boost::bind(&NavigationNode::dynamicParamCallback, this, _1, _2));

        // 初始化发布器和订阅器
        can_pub_ = nh_.advertise<can_msgs::CanFrame>("can_tx", 10);
        cmd_vel_sub_ = nh_.subscribe("cmd_vel", 1, &NavigationNode::cmdVelCallback, this);

        ROS_INFO("Navigation node initialized. CAN ID: 0x%X, DLC: %d", can_id_, can_dlc_);
    }

private:
    void loadParameters() {
        nh_.param<uint32_t>("can_id", can_id_, 0x200);
        nh_.param<int>("can_dlc", can_dlc_, 8);
        nh_.param<float>("max_linear_speed", max_linear_speed_, 1.0);
        nh_.param<float>("max_angular_speed", max_angular_speed_, 1.0);
    }

    void dynamicParamCallback(navigation::CanConfig& config, uint32_t level) {
        // 动态更新参数
        max_linear_speed_ = config.max_linear_speed;
        max_angular_speed_ = config.max_angular_speed;
        ROS_INFO("Updated dynamic params: max_linear=%.2f, max_angular=%.2f",
                 max_linear_speed_, max_angular_speed_);
    }

    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        // 限速处理
        float vx = std::clamp(msg->linear.x, -max_linear_speed_, max_linear_speed_);
        float wz = std::clamp(msg->angular.z, -max_angular_speed_, max_angular_speed_);

        // 模式判断
        Chassis_mode mode = (fabs(vx) > 0.001 || fabs(wz) > 0.001) ? chassis_run : chassis_stop;

        // 构造控制命令
        Cmd_chassis control_msg;
        control_msg.mode = mode;
        control_msg.vx = vx;
        control_msg.wz = wz;

        // 打包CAN帧
        can_msgs::CanFrame can_frame;
        can_frame.id = can_id_;
        can_frame.dlc = can_dlc_;
        std::memcpy(can_frame.data.data(), &control_msg, sizeof(Cmd_chassis));

        // 检查发布者有效性
        if (can_pub_.getNumSubscribers() > 0) {
            can_pub_.publish(can_frame);
            ROS_DEBUG_THROTTLE(1.0, "Sent CAN: mode=%d vx=%.2f wz=%.2f",
                               control_msg.mode, control_msg.vx, control_msg.wz);
        } else {
            ROS_WARN_THROTTLE(5.0, "No CAN subscribers! Check can_tx topic.");
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "navigation_node");
    NavigationNode node;
    ros::spin();
    return 0;
}