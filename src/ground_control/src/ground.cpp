#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <ground_control/CmdPose.h>
#include <map>
#include <vector>
#include <cmath>
#include <algorithm>

class GroundControl {
private:
    ros::NodeHandle nh_;
    int drone_num_;
    int cmd_num_;
    std::vector<int> drone_ids_;
    
    // 存储无人机当前位置
    std::map<int, geometry_msgs::PoseStamped> drone_positions_;
    
    // 目标位置发布器
    std::map<int, ros::Publisher> cmd_publishers_;
    
    // 目标位置序列
    std::vector<std::vector<geometry_msgs::Point>> formation_targets_;
    
    // 当前目标索引
    int current_target_index_;
    
    // 无人机到达状态
    std::map<int, bool> drone_arrival_status_;
    
    // 到达容差 (米)
    const float ARRIVAL_TOLERANCE;
    
    // 目标位置解析函数
    void parseFormationTargets(const std::vector<float>& targets) {
        // 清空现有目标
        formation_targets_.clear();
        
        // 检查是否有足够的目标点
        if (targets.empty()) {
            ROS_ERROR("No targets provided!");
            return;
        }
        
        // 每个无人机有 cmd_num 个目标点
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
                
                // 关键边界检查
                if (start_index + 2 >= targets.size()) {
                    ROS_ERROR("Invalid target index for drone %d, target %d", i, j);
                    break;
                }
                
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
        // 检查有效索引
        if (drone_id < 1 || drone_id > drone_num_) {
            ROS_WARN("Invalid drone_id: %d", drone_id);
            return false;
        }
        
        if (current_target_index_ >= cmd_num_) 
            return true;
        
        // 检查是否有位置数据
        if (drone_positions_.find(drone_id) == drone_positions_.end()) {
            ROS_WARN("No position data for drone %d", drone_id);
            return false;
        }
        
        const auto& current_pos = drone_positions_[drone_id].pose.position;
        
        // 检查目标索引有效性
        if (formation_targets_.size() <= drone_id - 1 || 
            formation_targets_[drone_id - 1].size() <= current_target_index_) {
            ROS_WARN("Invalid target index for drone %d", drone_id);
            return false;
        }
        
        const auto& target = formation_targets_[drone_id - 1][current_target_index_];
        
        float dx = current_pos.x - target.x;
        float dy = current_pos.y - target.y;
        float dz = current_pos.z - target.z;
        
        return (dx*dx + dy*dy + dz*dz) < (ARRIVAL_TOLERANCE * ARRIVAL_TOLERANCE);
    }
    
    // 检查所有无人机是否到达
    bool allDronesArrived() {
        for (int id : drone_ids_) {
            if (!checkArrival(id)) {
                return false;
            }
        }
        return true;
    }
    
    // 发送目标位置
    void sendTargets() {
        for (int id : drone_ids_) {
            if (current_target_index_ < cmd_num_) {
                // 检查目标索引有效性
                if (formation_targets_.size() <= id - 1 || 
                    formation_targets_[id - 1].size() <= current_target_index_) {
                    ROS_ERROR("Invalid target for drone %d, index %d", id, current_target_index_);
                    continue;
                }
                
                const auto& target = formation_targets_[id - 1][current_target_index_];
                
                ground_control::CmdPose cmd;
                cmd.x = target.x;
                cmd.y = target.y;
                cmd.z = target.z;
                
                if (cmd_publishers_.find(id) != cmd_publishers_.end()) {
                    cmd_publishers_[id].publish(cmd);
                } else {
                    ROS_ERROR("No publisher for drone %d", id);
                }
            }
        }
    }
    
    // 显示状态信息
    void displayStatus() {
        ROS_INFO("===== Formation Control Status [Target %d/%d] =====", 
                 current_target_index_ + 1, cmd_num_);
        
        for (int id : drone_ids_) {
            // 检查是否有位置数据
            if (drone_positions_.find(id) == drone_positions_.end()) {
                ROS_INFO("Drone %d: NO POSITION DATA", id);
                continue;
            }
            
            const auto& pos = drone_positions_[id].pose.position;
            bool arrived = checkArrival(id);
            
            if (current_target_index_ < cmd_num_) {
                // 检查目标索引有效性
                if (formation_targets_.size() <= id - 1 || 
                    formation_targets_[id - 1].size() <= current_target_index_) {
                    ROS_INFO("Drone %d: INVALID TARGET", id);
                } else {
                    const auto& target = formation_targets_[id - 1][current_target_index_];
                    
                    float distance = sqrt(
                        pow(target.x - pos.x, 2) + 
                        pow(target.y - pos.y, 2) + 
                        pow(target.z - pos.z, 2)
                    );
                    
                    ROS_INFO("Drone %d: %s | Pos(%.1f,%.1f,%.1f) -> Target(%.1f,%.1f,%.1f) | Dist: %.2f m", 
                             id, 
                             arrived ? "ARRIVED" : "MOVING",
                             pos.x, pos.y, pos.z,
                             target.x, target.y, target.z,
                             distance);
                }
            } else {
                ROS_INFO("Drone %d: MISSION COMPLETE | Pos(%.1f,%.1f,%.1f)", 
                         id, pos.x, pos.y, pos.z);
            }
        }
        
        if (current_target_index_ < cmd_num_) {
            ROS_INFO("Waiting for all drones to arrive at target %d...", current_target_index_ + 1);
        } else {
            ROS_INFO("===== MISSION COMPLETED =====");
        }
        
        ROS_INFO("=================================================");
    }

public:
    GroundControl() : 
        drone_num_(0), 
        cmd_num_(0), 
        current_target_index_(0),
        ARRIVAL_TOLERANCE(0.5)  // 初始化常量
    {
        // 获取参数
        ros::NodeHandle private_nh("~");
        
        // 获取无人机数量
        if (!private_nh.getParam("drone_num", drone_num_)) {
            ROS_ERROR("Failed to get drone_num parameter");
            drone_num_ = 0;
        } else {
            ROS_INFO("Got drone_num: %d", drone_num_);
        }
        
        // 获取编队数量
        if (!private_nh.getParam("cmd_num", cmd_num_)) {
            ROS_ERROR("Failed to get cmd_num parameter");
            cmd_num_ = 0;
        } else {
            ROS_INFO("Got cmd_num: %d", cmd_num_);
        }
        
        // 获取目标位置参数
        std::vector<float> targets;
        if (!private_nh.getParam("cmd_pose", targets)) {
            ROS_ERROR("Failed to get cmd_pose parameter");
        } else {
            ROS_INFO("Got %lu target values", targets.size());
        }
        
        // 解析目标位置
        parseFormationTargets(targets);
        
        // 生成无人机ID列表
        for (int i = 1; i <= drone_num_; ++i) {
            drone_ids_.push_back(i);
            drone_arrival_status_[i] = false;
        }
        
        // 创建位置订阅器
        for (int id : drone_ids_) {
            std::string topic_name = "/uav_" + std::to_string(id) + "/mavros/local_position/pose";
            ros::Subscriber sub = nh_.subscribe<geometry_msgs::PoseStamped>(
                topic_name, 10, 
                [this, id](const geometry_msgs::PoseStamped::ConstPtr& msg) {
                    if (msg) {
                        this->positionCallback(id, msg);
                    } else {
                        ROS_WARN("Received null message for drone %d", id);
                    }
                }
            );
            ROS_INFO("Subscribed to: %s", topic_name.c_str());
        }
        
        // 创建目标位置发布器
        for (int id : drone_ids_) {
            std::string topic_name = "/uav_" + std::to_string(id) + "/cmd_pose";
            cmd_publishers_[id] = nh_.advertise<ground_control::CmdPose>(topic_name, 10);
            ROS_INFO("Publisher created for: %s", topic_name.c_str());
        }
        
        ROS_INFO("Ground control initialized with %d drones and %d formation targets", 
                 drone_num_, cmd_num_);
    }
    
    // 位置回调函数
    void positionCallback(int drone_id, const geometry_msgs::PoseStamped::ConstPtr& msg) {
        drone_positions_[drone_id] = *msg;
    }
    
    // 运行主循环
    void run() {
        if (drone_num_ == 0 || cmd_num_ == 0) {
            ROS_ERROR("Invalid configuration! drone_num=%d, cmd_num=%d", drone_num_, cmd_num_);
            return;
        }
        
        ros::Rate rate(10);  // 10Hz更新频率
        
        // 初始发送第一个目标
        sendTargets();
        
        while (ros::ok()) {
            // 检查所有无人机是否到达当前目标
            if (current_target_index_ < cmd_num_ && allDronesArrived()) {
                ROS_INFO("All drones arrived at target %d. Moving to next target...", 
                         current_target_index_ + 1);
                
                // 移动到下一个目标
                current_target_index_++;
                
                // 发送新目标
                if (current_target_index_ < cmd_num_) {
                    sendTargets();
                } else {
                    ROS_INFO("Mission completed! All targets reached.");
                }
            }
            
            // 显示状态信息
            displayStatus();
            
            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "ground_control_node");
    
    try {
        GroundControl ground_control;
        ground_control.run();
    } catch (const std::exception& e) {
        ROS_FATAL("Exception in ground_control: %s", e.what());
    } catch (...) {
        ROS_FATAL("Unknown exception in ground_control");
    }
    
    return 0;
}