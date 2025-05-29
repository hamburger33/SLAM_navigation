#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class GoalListener {
public:
    GoalListener() : ac_("move_base", true) {
        // 初始化 ActionClient（非阻塞）
        ros::NodeHandle nh;
        sub_ = nh.subscribe("move_base_simple/goal", 1, &GoalListener::goalCallback, this);

        // 异步等待服务器（可设置超时）
        if (!ac_.waitForServer(ros::Duration(5.0))) {
            ROS_ERROR("move_base action server not available!");
            ros::shutdown();
        }
    }

    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        // 检查坐标系是否合法
        if (msg->header.frame_id != "map" && msg->header.frame_id != "odom") {
            ROS_WARN("Invalid frame_id: %s. Use 'map' or 'odom'.", msg->header.frame_id.c_str());
            return;
        }

        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose = *msg;  // 直接复用消息中的 header 和 pose

        // 发送目标并注册回调
        ac_.sendGoal(goal,
                     boost::bind(&GoalListener::doneCallback, this, _1, _2),
                     boost::bind(&GoalListener::activeCallback, this),
                     boost::bind(&GoalListener::feedbackCallback, this, _1));

        ROS_INFO("Goal sent to move_base: [%.2f, %.2f]",
                 msg->pose.position.x, msg->pose.position.y);
    }

private:
    // 导航完成回调
    void doneCallback(const actionlib::SimpleClientGoalState& state,
                      const move_base_msgs::MoveBaseResultConstPtr& result) {
        if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Goal reached!");
        } else {
            ROS_WARN("Failed to reach goal: %s", state.toString().c_str());
        }
    }

    // 导航激活回调（目标开始执行）
    void activeCallback() {
        ROS_DEBUG("Goal is now being processed.");
    }

    // 导航反馈回调（可选）
    void feedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) {
        ROS_DEBUG("Current position: [%.2f, %.2f]",
                  feedback->base_position.pose.position.x,
                  feedback->base_position.pose.position.y);
    }

    MoveBaseClient ac_;
    ros::Subscriber sub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "goal_listener");
    GoalListener listener;
    ros::spin();
    return 0;
}