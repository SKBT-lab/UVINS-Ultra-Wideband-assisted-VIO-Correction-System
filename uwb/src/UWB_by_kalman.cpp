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
const char *dev  = "/dev/ttyACM2";
geometry_msgs::Vector3 dis_msgs;
queue<float> filter_window_A0;
queue<float> filter_window_A1;
queue<float> filter_window_A2;
int filter_size = 10;


// Define state vector and observation vector
Vector3d x; // State vector
Vector3d z; // Observation vector

// Define matrices
Matrix3d A; // State transition matrix
Matrix3d H; // Measurement matrix
Matrix3d Q; // Process noise covariance matrix
Matrix3d R; // Measurement noise covariance matrix
Matrix3d P; // Estimate error covariance matrix

// Initialize matrices
//x.setZero(); // Initial state
//z.setRandom(); // Initial observation


float smooth_filter(float &Data, queue<float> &filter_window){
    if(filter_window.size() == filter_size){
      filter_window.pop();
    }
    filter_window.push(Data);
    float sum = 0.0;
    queue<float> temp_window = filter_window;
    for (int i = 0; i < filter_window.size(); i++){
       sum += temp_window.front();
       temp_window.pop();
    }
    float output = sum / filter_window.size();
    return output;
}

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



void renew_data(serialPort& myserial){
  while (true)
    {
      //nwrite = myserial.writeBuffer( buff, 8);
      myserial.readBuffer(buff, 1);
      if(buff[0] == '\n'){
        break;
      }
    }
}

int hex2int(char c){
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
{ serialPort myserial;
int nread,nwrite;
  cout<<"UWB Start"<<endl;
  myserial.OpenPort(dev);
  myserial.setup(19200,0,8,1,'N'); 
  A << 1, 0, 0,
       0, 1, 0,
       0, 0, 1; // Identity matrix
  H << 1, 0, 0,
       0, 1, 0,
       0, 0, 1; // Identity matrix
  Q << 0.1, 0, 0,
       0, 0.1, 0,
       0, 0, 0.1; // Process noise covariance
  R << 0.1, 0, 0,
       0, 0.1, 0,
       0, 0, 0.1; // Measurement noise covariance
  P = Matrix3d::Identity(); // Initial estimate error covariance

   //ros节点初始化
  ros::init(argc, argv, "uwb_module");

    //创建节点句柄
  ros::NodeHandle Node;
    
    //创建一个publisher，发布一个名为/turtle1/cmd_vel的topic, 消息类型为geometry_msgs::Twist， 队列长度为10
  ros::Publisher send_dis = Node.advertise<geometry_msgs::Vector3>("/UWB_module/distance", 10);

    //设置循环的频率
  ros::Rate loop_rate(10);

  int count = 0;

  renew_data(myserial);
  while (true)
  {
    myserial.readBuffer(buff, 12);
    if(buff[0] != 'm'){
      renew_data(myserial);
      continue;
    }
    
    //printf("%.*s",12 ,buff);
    if(buff[1] == 'a'){
      dis_T0_A0 = hex2int(buff[10]) + 16 * hex2int(buff[9]) + 16 * 16 * hex2int(buff[8]);
      //cout << "T0_A0: "<< dis_T0_A0 << endl;
      if(++count == 3){
        count = 0;
        z(0) = dis_T0_A0 / 1000;
        z(1) = dis_T0_A1 / 1000;
        z(2) = dis_T0_A2 / 1000;
        Kalman_filter();
        dis_msgs.x = x[0];
        dis_msgs.y = x[1];
        dis_msgs.z = x[2];
        send_dis.publish(dis_msgs);
        //loop_rate.sleep();
        ROS_INFO("Publish distance[T0-A0: %f m, T0-A1: %f m, T0-A2: %f m]", dis_msgs.x, dis_msgs.y, dis_msgs.z);
      }
    }

    else if(buff[1] == 'b'){
      dis_T0_A1 = hex2int(buff[10]) + 16 * hex2int(buff[9]) + 16 * 16 * hex2int(buff[8]);
      //cout << "T0_A1: "<< dis_T0_A1 << endl;
      if(++count == 3){
        count = 0;
        z(0) = dis_T0_A0 / 1000;
        z(1) = dis_T0_A1 / 1000;
        z(2) = dis_T0_A2 / 1000;
        Kalman_filter();
        dis_msgs.x = x[0];
        dis_msgs.y = x[1];
        dis_msgs.z = x[2];
        send_dis.publish(dis_msgs);
        //loop_rate.sleep();
        ROS_INFO("Publish distance[T0-A0: %f m, T0-A1: %f m, T0-A2: %f m]", dis_msgs.x, dis_msgs.y, dis_msgs.z);
      }
    }

    else if(buff[1] == 'c'){
      dis_T0_A2 = hex2int(buff[10]) + 16 * hex2int(buff[9]) + 16 * 16 * hex2int(buff[8]);
    
      //cout << "T0_A2: "<< dis_T0_A2 << endl;
      if(++count == 3){
        count = 0;
        z(0) = dis_T0_A0 / 1000;
        z(1) = dis_T0_A1 / 1000;
        z(2) = dis_T0_A2 / 1000;
        Kalman_filter();
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
