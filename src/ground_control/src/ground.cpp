#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <ground_control/CmdPose.h>
#include <map>
#include <vector>
#include <cmath>

class GroundStation {
private:
    ros::NodeHandle nh_;
    int drone_num_;
    std::vector<int> drone_ids_;
    
    // 存储无人机当前位置
    std::map<int, geometry_msgs::PoseStamped> drone_positions_;
    
    // 目标位置发布器
    std::map<int, ros::Publisher> cmd_publishers_;
    
    // 目标位置存储
    std::map<int, ground_control::CmdPose> target_positions_;
    
    // 目标位置计算函数
    void calculateTargets() {
        // 示例策略：形成等边三角形编队
        const float formation_radius = 5.0;  // 编队半径
        const float center_x = 10.0;        // 编队中心x坐标
        const float center_y = 10.0;         // 编队中心y坐标
        const float altitude = 15.0;         // 飞行高度
        
        for (size_t i = 0; i < drone_ids_.size(); ++i) {
            int drone_id = drone_ids_[i];
            float angle = 2 * M_PI * i / drone_ids_.size();
            
            target_positions_[drone_id].x = center_x + formation_radius * cos(angle);
            target_positions_[drone_id].y = center_y + formation_radius * sin(angle);
            target_positions_[drone_id].z = altitude;
        }
    }

public:
    GroundStation() : drone_num_(0) {
        // 获取无人机数量参数
        ros::NodeHandle private_nh("~");
        if (!private_nh.getParam("drone_num", drone_num_)) {
            ROS_ERROR("Failed to get drone_num parameter");
            drone_num_ = 0;
        }
        
        // 生成无人机ID列表
        for (int i = 1; i <= drone_num_; ++i) {
            drone_ids_.push_back(i);
        }
        
        // 初始化无人机位置存储
        for (int id : drone_ids_) {
            geometry_msgs::PoseStamped init_pose;
            init_pose.pose.position.x = 0;
            init_pose.pose.position.y = 0;
            init_pose.pose.position.z = 0;
            drone_positions_[id] = init_pose;
            
            // 初始化目标位置
            ground_control::CmdPose init_target;
            init_target.x = 0;
            init_target.y = 0;
            init_target.z = 0;
            target_positions_[id] = init_target;
        }
        
        // 创建位置订阅器
        for (int id : drone_ids_) {
            std::string topic_name = "/uav_" + std::to_string(id) + "/mavros/local_position/pose";
            ros::Subscriber sub = nh_.subscribe<geometry_msgs::PoseStamped>(
                topic_name, 10, 
                [this, id](const geometry_msgs::PoseStamped::ConstPtr& msg) {
                    this->positionCallback(id, msg);
                }
            );
        }
        
        // 创建目标位置发布器
        for (int id : drone_ids_) {
            std::string topic_name = "/uav_" + std::to_string(id) + "/cmd_pose";
            cmd_publishers_[id] = nh_.advertise<ground_control::CmdPose>(topic_name, 10);
        }
        
        ROS_INFO("Ground station initialized with %d drones", drone_num_);
    }
    
    // 位置回调函数
    void positionCallback(int drone_id, const geometry_msgs::PoseStamped::ConstPtr& msg) {
        drone_positions_[drone_id] = *msg;
        
        // 调试信息：显示无人机位置
        ROS_DEBUG("Drone %d position: (%.2f, %.2f, %.2f)", 
                 drone_id, 
                 msg->pose.position.x, 
                 msg->pose.position.y, 
                 msg->pose.position.z);
    }
    
    // 运行主循环
    void run() {
        ros::Rate rate(10);  // 10Hz更新频率
        
        while (ros::ok()) {
            // 计算新的目标位置
            calculateTargets();
            
            // 发布目标位置
            for (int id : drone_ids_) {
                cmd_publishers_[id].publish(target_positions_[id]);
            }
            
            // 显示状态信息
            displayStatus();
            
            ros::spinOnce();
            rate.sleep();
        }
    }
    
    // 显示状态信息
    void displayStatus() {
        ROS_INFO("===== Ground Station Status =====");
        ROS_INFO("Controlling %d drones", drone_num_);
        
        for (int id : drone_ids_) {
            const auto& pos = drone_positions_[id].pose.position;
            const auto& target = target_positions_[id];
            
            // 计算到目标的距离
            float distance = sqrt(
                pow(target.x - pos.x, 2) + 
                pow(target.y - pos.y, 2) + 
                pow(target.z - pos.z, 2)
            );
            
            ROS_INFO("Drone %d: Pos(%.1f,%.1f,%.1f) -> Target(%.1f,%.1f,%.1f) | Dist: %.2f m", 
                     id, 
                     pos.x, pos.y, pos.z,
                     target.x, target.y, target.z,
                     distance);
        }
        ROS_INFO("================================");
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "ground_control_node");
    GroundStation ground_control;
    ground_control.run();
    return 0;
}