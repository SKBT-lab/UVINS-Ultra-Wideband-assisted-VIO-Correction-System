#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>
#include <Eigen/Dense>
#include "uwb/UWB_msg.h"

using namespace Eigen;
std::ofstream outfile1;
std::ofstream outfile2;
std::ofstream outfile3;
std::ofstream outfile4;
std::ofstream outfile5;
Vector3d A0(-3.507, 1.671, 1.847);
Vector3d A1(2.011, -1.948, 1.865);
Vector3d A2(2.228, 1.485, 1.503);
Vector3d UWB_IMU(0.004, -0.0894, 0.0512);

void GetVIO(const nav_msgs::Odometry& msg){
    // 将位置信息写入文件
    outfile1 << std::fixed << std::setprecision(6)<< msg.header.stamp.toSec() << " " 
            << msg.pose.pose.position.x << " " 
            << msg.pose.pose.position.y << " " 
            << msg.pose.pose.position.z << std::endl;
    ROS_INFO("XY Fusion Write Successfully");    
}

void GetVIO2(const nav_msgs::Odometry& msg){
    // 将位置信息写入文件
    outfile2  << std::fixed << std::setprecision(6) << msg.header.stamp.toSec() << " " 
            << msg.pose.pose.position.x << " " 
            << msg.pose.pose.position.y << " " 
            << msg.pose.pose.position.z << std::endl;
    ROS_INFO("VINS Fusion Write Successfully");    
}

void Getreal(const geometry_msgs::PoseStamped& msg){

    if(msg.pose.position.x < 100000 && msg.pose.position.y < 100000 && msg.pose.position.z < 100000){
    // 将位置信息写入文件
        Quaterniond q;
        q.x() = msg.pose.orientation.x;
        q.y() = msg.pose.orientation.y;
        q.z() = msg.pose.orientation.z;
        q.w() = msg.pose.orientation.w;
        Vector3d p(msg.pose.position.x / 1000, msg.pose.position.y / 1000, msg.pose.position.z / 1000);

        Vector3d T0A0 = q * UWB_IMU + p - A0;
        Vector3d T0A1 = q * UWB_IMU + p - A1;
        Vector3d T0A2 = q * UWB_IMU + p - A2;

        outfile5 << std::fixed << std::setprecision(6) << msg.header.stamp.toSec() << " " 
                    << T0A0.norm() << " " 
                    << T0A1.norm() << " " 
                    << T0A2.norm() << std::endl;
        outfile3 << std::fixed << std::setprecision(6) << msg.header.stamp.toSec() << " " 
                << msg.pose.position.x / 1000 << " " 
                << msg.pose.position.y / 1000 << " " 
                << (msg.pose.position.z - 183.5) / 1000 << std::endl;
        ROS_INFO("Real Write Successfully");    
    }
}

void GetUWB(const geometry_msgs::PoseStamped& msg){
    if(msg.pose.position.x < 100000 && msg.pose.position.y < 100000 && msg.pose.position.z < 100000){
    // 将位置信息写入文件
        outfile4 << std::fixed << std::setprecision(6) << msg.header.stamp.toSec() << " " 
                << msg.pose.position.x  << " " 
                << msg.pose.position.y  << " " 
                << msg.pose.position.z  << std::endl;
        ROS_INFO("UWB Write Successfully");    
    }
}

void GetT_A(const uwb::UWB_msg& msg){
    if(msg.D0 > 0.2 && msg.D1 > 0.2 && msg.D2 > 0.2){
    // 将位置信息写入文件
        outfile5 << std::fixed << std::setprecision(6) << msg.stamp.toSec() << " " 
                << msg.D0  << " " 
                << msg.D1  << " " 
                << msg.D2  << std::endl;
        ROS_INFO("TA Write Successfully");    
    }
}


int main(int argc, char** argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "write_to_txt_node");
    ros::NodeHandle nh;

    // 创建文件输出流
    outfile1.open("/home/skbt/VIU_ROSBAG/txt_Data/VINS_UWB.txt", std::ios::out);
    outfile2.open("/home/skbt/VIU_ROSBAG/txt_Data/VINS.txt", std::ios::out);
    outfile3.open("/home/skbt/VIU_ROSBAG/txt_Data/Real.txt", std::ios::out);
    outfile4.open("/home/skbt/VIU_ROSBAG/txt_Data/UWB.txt", std::ios::out);
    outfile5.open("/home/skbt/VIU_ROSBAG/txt_Data/T_A.txt", std::ios::out);
    // 创建订阅者
    ros::Subscriber VIO_sub = nh.subscribe("/vins_fusion/imu_propagate", 10, GetVIO);
    ros::Subscriber VIO_sub2 = nh.subscribe("/vins_fusion2/imu_propagate2", 10, GetVIO2);
    ros::Subscriber real_sub = nh.subscribe("/vrpn_client_node/XY_Bigbaby/pose", 10, Getreal);
    ros::Subscriber UWB_sub = nh.subscribe("/UWB_module/optimization_result", 10, GetUWB);
    ros::Subscriber T_A_sub = nh.subscribe("/UWB_module/distance", 10, GetT_A);
   ///vrpn_client_node/bigbaby/pose
    // 循环等待ROS消息
    ros::spin();

    // 关闭文件输出流
    outfile1.close();
    outfile2.close();
    outfile3.close();
    outfile4.close();
    outfile5.close();

    return 0;
}