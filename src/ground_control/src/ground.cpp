#include <geometry_msgs/PoseStamped.h>
#include <ground_control/CmdPose.h>
#include <ros/ros.h>
#include <algorithm>
#include <boost/bind.hpp>
#include <cmath>
#include <map>
#include <mutex>
#include <vector>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

class GroundControl {
   private:
    ros::NodeHandle nh_;
    int drone_num_;
    int cmd_num_;
    std::vector<int> drone_ids_;

    // 存储无人机当前位置
    std::map<int, geometry_msgs::PoseStamped> drone_positions_;
    std::mutex position_mutex_;  // 互斥锁保护位置数据

    // 目标位置发布器
    std::map<int, ros::Publisher> cmd_publishers_;

    // 持久化的位置订阅者
    std::map<int, ros::Subscriber> pose_subscribers_;

    // 目标位置序列
    std::vector<std::vector<geometry_msgs::Point>> formation_targets_;

    // 当前目标索引
    int current_target_index_;

    // 到达容差 (米)
    const float ARRIVAL_TOLERANCE;

    // 目标位置解析函数
    void parseFormationTargets(const std::vector<float>& targets) {
        formation_targets_.clear();
        if (targets.empty()) {
            ROS_ERROR("No targets provided!");
            return;
        }
        int points_per_drone = cmd_num_ * 3;
        int required_size = drone_num_ * points_per_drone;
        if (targets.size() < required_size) {
            ROS_ERROR("Insufficient targets! Got %lu, need %d", targets.size(), required_size);
            return;
        }
        for (int i = 0; i < drone_num_; ++i) {
            std::vector<geometry_msgs::Point> drone_targets;
            for (int j = 0; j < cmd_num_; ++j) {
                int start_index = i * points_per_drone + j * 3;
                geometry_msgs::Point target;
                target.x = targets[start_index];
                target.y = targets[start_index + 1];
                target.z = targets[start_index + 2];
                drone_targets.push_back(target);
            }
            formation_targets_.push_back(drone_targets);
        }
        ROS_INFO("Parsed %d formation targets for %d drones", cmd_num_, drone_num_);
    }

    // 检查无人机是否到达目标
    bool checkArrival(int drone_id) {
        if (drone_id < 1 || drone_id > drone_num_) {
            ROS_WARN("Invalid drone_id: %d", drone_id);
            return false;
        }
        if (current_target_index_ >= cmd_num_)
            return true;

        std::lock_guard<std::mutex> lock(position_mutex_);
        if (drone_positions_.find(drone_id) == drone_positions_.end()) {
            ROS_WARN_THROTTLE(5.0, "No position data for drone %d", drone_id);
            return false;
        }

        const auto& current_pos = drone_positions_[drone_id].pose.position;
        if (formation_targets_.size() <= static_cast<size_t>(drone_id - 1) ||
            formation_targets_[drone_id - 1].size() <= static_cast<size_t>(current_target_index_)) {
            ROS_WARN_THROTTLE(5.0, "Invalid target index for drone %d", drone_id);
            return false;
        }

        const auto& target = formation_targets_[drone_id - 1][current_target_index_];
        float dx = current_pos.x - target.x;
        float dy = current_pos.y - target.y;
        float dz = current_pos.z - target.z;
        return (dx * dx + dy * dy + dz * dz) < (ARRIVAL_TOLERANCE * ARRIVAL_TOLERANCE);
    }

    // 检查所有无人机是否到达
    bool allDronesArrived() {
        for (int id : drone_ids_) {
            if (!checkArrival(id))
                return false;
        }
        return true;
    }

    // 发送目标位置
    void sendTargets() {
        for (int id : drone_ids_) {
            if (current_target_index_ < cmd_num_) {
                if (formation_targets_.size() <= static_cast<size_t>(id - 1) ||
                    formation_targets_[id - 1].size() <= static_cast<size_t>(current_target_index_)) {
                    ROS_ERROR("Invalid target for drone %d, index %d", id, current_target_index_);
                    continue;
                }

                const auto& target = formation_targets_[id - 1][current_target_index_];
                ground_control::CmdPose cmd;
                cmd.x = target.x;
                cmd.y = target.y;
                cmd.z = target.z;
                cmd_publishers_[id].publish(cmd);
                // ROS_INFO("Sent target to drone %d: (%.1f, %.1f, %.1f)", id, cmd.x, cmd.y, cmd.z);
            }
        }
    }

    // 位置回调函数
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, int drone_id) {
        if (!msg) {
            ROS_WARN("Received null pose for drone %d", drone_id);
            return;
        }

        std::lock_guard<std::mutex> lock(position_mutex_);
        drone_positions_[drone_id] = *msg;
        ROS_DEBUG("Received pose for drone %d: (%.2f, %.2f, %.2f)",
                  drone_id,
                  msg->pose.position.x,
                  msg->pose.position.y,
                  msg->pose.position.z);
    }

   public:
    GroundControl()
        : drone_num_(0), cmd_num_(0), current_target_index_(0), ARRIVAL_TOLERANCE(0.5) {
        ros::NodeHandle private_nh("~");
        private_nh.getParam("drone_num", drone_num_);
        private_nh.getParam("cmd_num", cmd_num_);

        std::vector<float> targets;
        private_nh.getParam("cmd_pose", targets);
        parseFormationTargets(targets);

        for (int i = 1; i <= drone_num_; ++i) {
            drone_ids_.push_back(i);
        }

        // 创建持久化的位置订阅者
        for (int id : drone_ids_) {
            std::string topic = "/uav" + std::to_string(id) + "/mavros/local_position/pose";
            pose_subscribers_[id] = nh_.subscribe<geometry_msgs::PoseStamped>(
                topic, 10, boost::bind(&GroundControl::poseCallback, this, _1, id));
            ROS_INFO("Subscribed to: %s", topic.c_str());
        }

        // 创建目标发布器
        for (int id : drone_ids_) {
            std::string topic = "/uav" + std::to_string(id) + "/cmd_pose";
            cmd_publishers_[id] = nh_.advertise<ground_control::CmdPose>(topic, 10);
            ROS_INFO("Publisher created for: %s", topic.c_str());
        }

        ROS_INFO("Ground control initialized with %d drones and %d formation targets",
                 drone_num_, cmd_num_);
    }

    void run() {
        if (!drone_num_ || !cmd_num_) {
            ROS_ERROR("Invalid drone_num(%d) or cmd_num(%d)!", drone_num_, cmd_num_);
            return;
        }

        ros::Rate rate(10);
        ROS_INFO("Sent initial targets to all drones");

        while (ros::ok()) {
            if (current_target_index_ < cmd_num_ && allDronesArrived()) {
                ROS_INFO("All drones arrived at target %d. Moving to next...",
                         current_target_index_ + 1);
                current_target_index_++;

                if (current_target_index_ < cmd_num_) {
                    sendTargets();
                } else {
                    ROS_INFO("Mission completed! All targets reached.");
                }
            }
            sendTargets();
            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "ground");
    GroundControl gc;
    gc.run();
    return 0;
}