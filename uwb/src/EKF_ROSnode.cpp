#include "EKF.h"
#include "uwb/UWB_msg.h"
#include <geometry_msgs/PoseStamped.h>

E_K_FILTER ekf;
geometry_msgs::PoseStamped pose_msg;
ros::Publisher EKF_pub;

void GetUWB_dis(const uwb::UWB_msg& uwb_dis){
   ekf.Input_UWB(uwb_dis.D0, uwb_dis.D1, uwb_dis.D2);
   ekf.ekf_predict(ekf.x_pv);
   ekf.ekf_update();
   pose_msg.header.stamp = ros::Time::now();
   pose_msg.pose.position.x = ekf.x_p[0];
   pose_msg.pose.position.y = ekf.x_p[1];
   pose_msg.pose.position.z = ekf.x_p[2];
   EKF_pub.publish(pose_msg);
   ROS_INFO("UWB result with EKF:[%f, %f, %f]", ekf.x_p[0], ekf.x_p[1], ekf.x_p[2]);
   
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "EKF");
  ros::NodeHandle n;
  EKF_pub = n.advertise<geometry_msgs::PoseStamped>("/UWB_module/EKF_result", 10);
  ros::Subscriber UWB_sub = n.subscribe("/UWB_module/distance", 10, GetUWB_dis);
  ros::spin(); 
}