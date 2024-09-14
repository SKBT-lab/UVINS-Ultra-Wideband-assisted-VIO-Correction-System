#include <iostream>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>
#include <vector>
#include <queue>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;


float dis_T0_A0, dis_T0_A1, dis_T0_A2;
geometry_msgs::Vector3 dis_msgs;
std_msgs::Float32 err;

float fc = 5;

float char2num(char c){
  if((c >= '0') && (c <= '9')){
    return c - '0';
  }
  else if ((c >= 'a') && (c <= 'e')){
    return c - 'a' + 10;
  }
  else{
    return 0;
  }
  
}

float correct(float &x){
  return 0.9678 * x - 0.154;
}

int main(int argc, char *argv[])
{
  double tau = 1.0 / (2 * M_PI * fc);
  double arf = tau / (tau + 1);
  int serial_port = open("/dev/ttyUSB0", O_RDWR);
    //ros节点初始化
  ros::init(argc, argv, "uwb_module_DT");

    //创建节点句柄
  ros::NodeHandle Node;
    
    //创建一个publisher，发布一个名为/turtle1/cmd_vel的topic, 消息类型为geometry_msgs::Twist， 队列长度为10
  ros::Publisher send_dis = Node.advertise<geometry_msgs::Vector3>("/UWB_module/distance_DT", 10);
  ros::Publisher send_err = Node.advertise<std_msgs::Float32>("/UWB_module/distance_errDT", 10);
    //设置循环的频率
  ros::Rate loop_rate(10);

    if (serial_port < 0) {
        std::cerr << "Error " << errno << " opening serial port: " << strerror(errno) << std::endl;
        return 1;
    }

    // Set serial port parameters
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(serial_port, &tty) != 0) {
        std::cerr << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
        return 1;
    }

    cfsetospeed(&tty, B460800);
    cfsetispeed(&tty, B460800);

    tty.c_cflag &= ~PARENB; // No parity
    tty.c_cflag &= ~CSTOPB; // 1 stop bit
    tty.c_cflag &= ~CSIZE; // Mask the character size bits
    tty.c_cflag |= CS8; // 8 data bits
    tty.c_cflag &= ~CRTSCTS; // Disable hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // Turn on the receiver and ignore modem control lines

    tty.c_cc[VMIN] = 0; // Minimum number of characters to read
    tty.c_cc[VTIME] = 10; // Time to wait for data (in tenths of a second)

    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        std::cerr << "Error " << errno << " from tcsetattr" << std::endl;
        return 1;
    }

    // Read n characters from serial port
    int n = 32; // Number of characters to read
    char buff[n + 1]; // Buffer to hold read data

    while (true) {
        int bytes_read = read(serial_port, buff, n);
        if (bytes_read < 0) {
            std::cerr << "Error " << errno << " reading serial port: " << strerror(errno) << std::endl;
            break;
        }
        if (bytes_read > 0) {
            buff[bytes_read] = '\0'; // Null terminate the buffer
            //std::cout << "Read " << bytes_read << " bytes: " << buf << std::endl;
            if (buff[0] == 'm')
            {
                dis_T0_A0 = (char2num(buff[13]) + 16 * char2num(buff[12]) + 16 * 16 * char2num(buff[11]) + 16 * 16 * 16 * char2num(buff[10])) / 1000;
                dis_T0_A1 = (char2num(buff[22]) + 16 * char2num(buff[21]) + 16 * 16 * char2num(buff[20]) + 16 * 16 * 16 * char2num(buff[19])) / 1000;
                dis_T0_A2 = (char2num(buff[31]) + 16 * char2num(buff[30]) + 16 * 16 * char2num(buff[29]) + 16 * 16 * 16 * char2num(buff[28])) / 1000;
                //if(dis_T0_A0 != 0 && dis_T0_A1 != 0 && dis_T0_A2 != 0){
                  if(dis_T0_A0 != 0){
                  dis_T0_A0 = correct(dis_T0_A0);
                  dis_T0_A0 = dis_T0_A0 + arf * (dis_msgs.x - dis_T0_A0);
                  err.data = dis_T0_A0 - 1.91;
                  dis_T0_A1 = correct(dis_T0_A1);
                  dis_T0_A2 = correct(dis_T0_A2);
                  dis_msgs.x = dis_T0_A0;
                  dis_msgs.y = dis_T0_A1;
                  dis_msgs.z = dis_T0_A2;
                  send_dis.publish(dis_msgs);
                  send_err.publish(err);
                //loop_rate.sleep();
                  ROS_INFO("Publish distance[T0-A0: %f m, T0-A1: %f m, T0-A2: %f m]", dis_msgs.x, dis_msgs.y, dis_msgs.z);
                }
            }
        }
    }

    close(serial_port);

    return 0;
}