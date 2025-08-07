#include <geometry_msgs/PoseStamped.h>
#include <ground_control/CmdPose.h>  // 使用统一的消息类型
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>

// 添加全局变量
geometry_msgs::Point target_position;
bool new_target_received = false;
mavros_msgs::State current_state;

void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

void target_cb(const ground_control::CmdPose::ConstPtr& msg) {
    target_position.x = msg->x;
    target_position.y = msg->y;
    target_position.z = msg->z;
    new_target_received = true;
    ROS_INFO("Received new target: (%.2f, %.2f, %.2f)",
             target_position.x, target_position.y, target_position.z);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "air_node");
    ros::NodeHandle nh;

    // 获取无人机ID参数
    int drone_id = 1;
    ros::param::param<int>("~drone_id", drone_id, 1);

    // 使用全局话题名称
    std::string cmd_topic = "/uav" + std::to_string(drone_id) + "/cmd_pose";
    ros::Subscriber target_sub = nh.subscribe<ground_control::CmdPose>(cmd_topic, 10, target_cb);

    // 订阅mavros状态
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);

    // 发布无人机位姿信息
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

    // 定义服务客户端
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    // 设置发布频率
    ros::Rate rate(20.0);

    // 等待FCU连接
    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    while (ros::ok() && drone_id == 6) {
        if (new_target_received && target_position.z > 0.5) {
            break;
        }
        ros::spinOnce();
        rate.sleep();
    }

    // 发送初始设置点
    for (int i = 100; ros::ok() && i > 0; --i) {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while (ros::ok()) {
        // 尝试切换到OFFBOARD模式
        if (current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(1.0))) {
            if (set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent) {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        // 尝试解锁无人机
        else if (!current_state.armed &&
                 (ros::Time::now() - last_request > ros::Duration(1.0))) {
            if (arming_client.call(arm_cmd) &&
                arm_cmd.response.success) {
                ROS_INFO("Vehicle armed");
                break;
            }
            last_request = ros::Time::now();
        }

        // 更新目标位置
        if (new_target_received) {
            pose.pose.position = target_position;
            new_target_received = false;
            ROS_INFO("Updating target position for drone %d", drone_id);
        }

        // 发布位置指令
        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    while (ros::ok()) {
        // 更新目标位置
        if (new_target_received) {
            pose.pose.position = target_position;
            new_target_received = false;
            ROS_INFO("Updating target position for drone %d", drone_id);
        }

        // 发布位置指令
        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}