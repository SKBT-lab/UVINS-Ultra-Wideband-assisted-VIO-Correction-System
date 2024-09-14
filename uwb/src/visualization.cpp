#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

  // 创建一个nav_msgs::Path类型的消息
nav_msgs::Path path_msg;
ros::Publisher path_pub;
nav_msgs::Path realpath_msg;
ros::Publisher realpath_pub;

void trajectoryCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  // 创建一个Path中的PoseStamped类型的消息
  geometry_msgs::PoseStamped pose;
  pose.header = msg->header;
  pose.pose.position.x = - msg->pose.position.z;
  pose.pose.position.y = msg->pose.position.y;
  pose.pose.position.z = msg->pose.position.x;

  // 将PoseStamped添加到Path消息中
  path_msg.poses.push_back(pose);
  // 创建一个Publisher发布可视化的路径消息
 
  // 发布可视化的路径消息
  path_pub.publish(path_msg);
  //ROS_INFO("Publish Path");
}

void realtrajectoryCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  // 将PoseStamped添加到Path消息中
  if(msg -> pose.position.x < 100000 && msg -> pose.position.y < 100000 && msg -> pose.position.z < 100000){
    geometry_msgs::PoseStamped pose;
    pose.header = msg->header;
    pose.pose.position.x = msg -> pose.position.x / 1000;
    pose.pose.position.y = msg -> pose.position.y / 1000;
    pose.pose.position.z = (msg -> pose.position.z - 193) / 1000;
    realpath_msg.poses.push_back(pose);
    // 发布可视化的路径消息
    realpath_pub.publish(realpath_msg);
    //ROS_INFO("Publish realPath");
  }
}



int main(int argc, char** argv)
{
  // 初始化ROS节点
  ros::init(argc, argv, "visualize_trajectory_node");
  ros::NodeHandle nh;
  path_msg.header.frame_id = "world"; // 设置坐标系
  realpath_msg.header.frame_id = "world"; // 设置坐标系
  path_pub = nh.advertise<nav_msgs::Path>("/orbslam3/path", 10);
  realpath_pub = nh.advertise<nav_msgs::Path>("/real_path", 10);
  // 创建一个Subscriber订阅三维位置坐标消息
  ros::Subscriber trajectory_sub = nh.subscribe<geometry_msgs::PoseStamped>("/orbslam3/vision_pose/pose", 10, trajectoryCallback);
  ros::Subscriber realtrajectory_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/XY_Bigbaby/pose", 10, realtrajectoryCallback);
  ros::spin();

  return 0;
}

