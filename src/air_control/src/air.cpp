#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
  current_state = *msg;
}

geometry_msgs::PoseStamped target_pose;
void cmd_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
  target_pose = *msg;
  ROS_INFO("[UAV %s] Received target: x=%.2f y=%.2f z=%.2f",
           ros::this_node::getNamespace().c_str(),
           msg->pose.position.x,
           msg->pose.position.y,
           msg->pose.position.z);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "air_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // 获取 UAV 编号
  std::string drone_id;
  pnh.param<std::string>("drone_id", drone_id, "1");
  std::string ns = "/uav_" + drone_id;

  // 订阅 MAVROS 状态
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>(
    "mavros/state", 10, state_cb);
  // 订阅地面站命令话题
  ros::Subscriber cmd_sub = nh.subscribe<geometry_msgs::PoseStamped>(
    ns + "/cmd_pose", 10, cmd_cb);
  // 发布本机位置给 MAVROS
  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(
    "mavros/setpoint_position/local", 10);
  
  // 服务客户端：arming、mode switch
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>(
    "mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(
    "mavros/set_mode");

  ros::Rate rate(20.0);
  // 等待连接
  while(ros::ok() && !current_state.connected){
    ros::spinOnce(); rate.sleep();
  }

  // 发送预设 setpoints
  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = 0;
  pose.pose.position.y = 0;
  pose.pose.position.z = 2;
  for(int i=0; i<100 && ros::ok(); ++i){
    local_pos_pub.publish(pose);
    ros::spinOnce(); rate.sleep();
  }

  // Offboard & Arm 请求
  mavros_msgs::SetMode offb_set;
  offb_set.request.custom_mode = "OFFBOARD";
  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;
  ros::Time last_req = ros::Time::now();

  // 主循环
  while(ros::ok()){
    // 切换模式
    if(current_state.mode != "OFFBOARD" &&
       ros::Time::now() - last_req > ros::Duration(5.0)){
      if(set_mode_client.call(offb_set) && offb_set.response.mode_sent)
        ROS_INFO("[UAV %s] OFFBOARD enabled", drone_id.c_str());
      last_req = ros::Time::now();
    } else if(!current_state.armed &&
              ros::Time::now() - last_req > ros::Duration(5.0)){
      if(arming_client.call(arm_cmd) && arm_cmd.response.success)
        ROS_INFO("[UAV %s] Armed", drone_id.c_str());
      last_req = ros::Time::now();
    }

    // 如果收到了目标位姿，就飞向它；否则保持初始姿态
    if(target_pose.header.stamp.sec != 0)
      local_pos_pub.publish(target_pose);
    else
      local_pos_pub.publish(pose);

    ros::spinOnce(); rate.sleep();
  }
  return 0;
}