#include "../include/uwb/Serial.hpp"
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <vector>
#include <queue>
#include <Eigen/Dense>


using namespace std;
using namespace Eigen;

uint8_t buff[100];
float dis_T0_A0, dis_T0_A1, dis_T0_A2;
geometry_msgs::Vector3 dis_msgs;
queue<float> filter_window_A0;
queue<float> filter_window_A1;
queue<float> filter_window_A2;
int filter_size = 10;


// Define state vector and observation vector
Vector3d x(2.0, 4.0, 5.0); // State vector
Vector3d x0(2.0, 4.0, 5.0);
Vector3d z; // Observation vector
Vector3d u(0, 0, 0);

// Define matrices
Matrix3d A; // State transition matrix
Matrix3d H; // Measurement matrix
Matrix3d Q; // Process noise covariance matrix
Matrix3d R; // Measurement noise covariance matrix
Matrix3d P; // Estimate error covariance matrix

// Initialize matrices
//x.setZero(); // Initial state
//z.setRandom(); // Initial observation


void Kalman_filter(){
    x = A * x;
    P = A * P * A.transpose() + Q;

// Update step
    Vector3d y = z - H * x;
    Matrix3d S = H * P * H.transpose() + R;
    Matrix3d K = P * H.transpose() * S.inverse();
    x = x + K * y;
    P = (Matrix3d::Identity() - K * H) * P;
}




int char2num(char c){
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


int main(int argc, char *argv[])
{ 
  A << 1, 0, 0,
       0, 1, 0,
       0, 0, 1; // Identity matrix
  H << 1, 0, 0,
       0, 1, 0,
       0, 0, 1; // Identity matrix
  Q << 0.01, 0, 0,
       0, 0.01, 0,
       0, 0, 0.01; // Process noise covariance
  R << 0.01, 0, 0,
       0, 0.01, 0,
       0, 0, 0.01; // Measurement noise covariance
  P = Matrix3d::Identity(); // Initial estimate error covariance

  //KalmanFilter<3, 3, 3> kf;
  //kf.init(x0, P, A, u, Q, H, R);

  int serial_port = open("/dev/ttyUSB0", O_RDWR);
   //ros节点初始化
  ros::init(argc, argv, "uwb_KM_module");

    //创建节点句柄
  ros::NodeHandle Node;
    
    //创建一个publisher，发布一个名为/turtle1/cmd_vel的topic, 消息类型为geometry_msgs::Twist， 队列长度为10
  ros::Publisher send_dis = Node.advertise<geometry_msgs::Vector3>("/UWB_module/distance_KM", 10);

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
  while (true)
  {
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
        z(0) = dis_T0_A0;
        z(1) = dis_T0_A1;
        z(2) = dis_T0_A2;
        Kalman_filter();
        //kf.predict();
        //kf.update(z);
        //x = kf.state();
        dis_msgs.x = x[0];
        dis_msgs.y = x[1];
        dis_msgs.z = x[2];
        send_dis.publish(dis_msgs);
        //loop_rate.sleep();
        ROS_INFO("Publish distance[T0-A0: %f m, T0-A1: %f m, T0-A2: %f m]", dis_msgs.x, dis_msgs.y, dis_msgs.z);
      }
    }
  }
}

