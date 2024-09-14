//
// Created by XingYu on 23-3-30.
//

#include <iostream>
#include <ceres/ceres.h>
#include <chrono>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <numeric>
#include <cmath>
#include <queue>
#include <Eigen/Dense>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;
using namespace Eigen;

double x_VIO, y_VIO, z_VIO, T0_A0, T0_A1, T0_A2;
double xe = 0.0, ye = 0.0, ze = 0.0;
double xyz[3] = {xe, ye, ze};
//A0净高107
//double A0[3] = {0.4, 4.4, 0.84};//2楼数据
//double A0[3] = {0, 2.6, 1.635};
double A0[3] = {5.5, -0.202, 1.635};
//double A1[3] = {5.2, 0, 0.55};
double A1[3] = {5.5, -3.902, 1.49};
double A2[3] = {-2.1, 3.838, 1.61}; //不准
int window_size_cor = 10;
int window_size_VIO = 3;
int flag = 0;
vector<double> cur_data(6); 
queue<vector<double>> cor_window;
queue<vector<double>> VIO_window;
vector<vector<double>> cor_vector;
vector<vector<double>> cov;
MatrixXd cov_Matrix(6, 6);
MatrixXd cov_ivs(6, 6);


geometry_msgs::PoseStamped result_msgs;
//ros::NodeHandle n;
ros::Publisher optmz_result;




ceres::Solver::Options options;     // 配置项
ceres::Solver::Summary summary;



// 代价函数的计算模型
struct FITTING_COST {
  FITTING_COST(double x_v, double y_v, double z_v, double T0_A0, double T0_A1, double T0_A2, MatrixXd Cov_i) : 
               xv(x_v), yv(y_v), zv(z_v), d0(T0_A0), d1(T0_A1), d2(T0_A2), cov_inv(Cov_i) {}
  // 残差的计算
  template<typename T>
  bool operator()(
    const T *const xyz, // 模型参数，有3维
    T *residual) const {
    residual[0] = 0.5 * (cov_inv(0, 0) * ceres::pow(T(xv) - xyz[0], 2)  
                  + cov_inv(1, 1) * ceres::pow(T(yv) - xyz[1], 2)  
                  + cov_inv(2, 2) * ceres::pow(T(zv) - xyz[2], 2)  
                  + cov_inv(3, 3) * ceres::pow((ceres::sqrt(ceres::pow(xyz[0] - A0[0], 2) + ceres::pow(xyz[1] - A0[1], 2) + ceres::pow(xyz[2] - A0[2], 2)) - d0), 2) 
                  //+ cov_inv(4, 4) * ceres::pow((ceres::sqrt(ceres::pow(xyz[0] - A1[0], 2) + ceres::pow(xyz[1] - A1[1], 2) + ceres::pow(xyz[2] - A1[2], 2)) - d1), 2) 
                  //+ cov_inv(5, 5) * ceres::pow((ceres::sqrt(ceres::pow(xyz[0] - A2[0], 2) + ceres::pow(xyz[1] - A2[1], 2) + ceres::pow(xyz[2] - A2[2], 2)) - d2), 2) 
                  //+ 2 * cov_inv(0, 1) * (T(xv) - xyz[0]) * (T(yv) - xyz[1]) 
                  //+ 2 * cov_inv(0, 2) * (T(xv) - xyz[0]) * (T(zv) - xyz[2]) 
                  //+ 2 * cov_inv(0, 3) * (T(xv) - xyz[0]) * (ceres::sqrt(ceres::pow(xyz[0] - A0[0], 2) + ceres::pow(xyz[1] - A0[1], 2) + ceres::pow(xyz[2] - A0[2], 2)) - d0) 
                  //+ 2 * cov_inv(0, 4) * (T(xv) - xyz[0]) * (ceres::sqrt(ceres::pow(xyz[0] - A1[0], 2) + ceres::pow(xyz[1] - A1[1], 2) + ceres::pow(xyz[2] - A1[2], 2)) - d1) 
                  //+ 2 * cov_inv(0, 5) * (T(xv) - xyz[0]) * (ceres::sqrt(ceres::pow(xyz[0] - A2[0], 2) + ceres::pow(xyz[1] - A2[1], 2) + ceres::pow(xyz[2] - A2[2], 2)) - d2) 
                  //+ 2 * cov_inv(1, 2) * (T(yv) - xyz[1]) * (T(zv) - xyz[2]) 
                  //+ 2 * cov_inv(1, 3) * (T(yv) - xyz[1]) * (ceres::sqrt(ceres::pow(xyz[0] - A0[0], 2) + ceres::pow(xyz[1] - A0[1], 2) + ceres::pow(xyz[2] - A0[2], 2)) - d0) 
                  //+ 2 * cov_inv(1, 4) * (T(yv) - xyz[1]) * (ceres::sqrt(ceres::pow(xyz[0] - A1[0], 2) + ceres::pow(xyz[1] - A1[1], 2) + ceres::pow(xyz[2] - A1[2], 2)) - d1) 
                  //+ 2 * cov_inv(1, 5) * (T(yv) - xyz[1]) * (ceres::sqrt(ceres::pow(xyz[0] - A2[0], 2) + ceres::pow(xyz[1] - A2[1], 2) + ceres::pow(xyz[2] - A2[2], 2)) - d2) 
                  //+ 2 * cov_inv(2, 3) * (T(zv) - xyz[2]) * (ceres::sqrt(ceres::pow(xyz[0] - A0[0], 2) + ceres::pow(xyz[1] - A0[1], 2) + ceres::pow(xyz[2] - A0[2], 2)) - d0) 
                  //+ 2 * cov_inv(2, 4) * (T(zv) - xyz[2]) * (ceres::sqrt(ceres::pow(xyz[0] - A1[0], 2) + ceres::pow(xyz[1] - A1[1], 2) + ceres::pow(xyz[2] - A1[2], 2)) - d1) 
                  //+ 2 * cov_inv(2, 5) * (T(zv) - xyz[2]) * (ceres::sqrt(ceres::pow(xyz[0] - A2[0], 2) + ceres::pow(xyz[1] - A2[1], 2) + ceres::pow(xyz[2] - A2[2], 2)) - d2) 
                  //+ 2 * cov_inv(3, 4) * (ceres::sqrt(ceres::pow(xyz[0] - A0[0], 2) + ceres::pow(xyz[1] - A0[1], 2) + ceres::pow(xyz[2] - A0[2], 2)) - d0) * 
                  //                     (ceres::sqrt(ceres::pow(xyz[0] - A1[0], 2) + ceres::pow(xyz[1] - A1[1], 2) + ceres::pow(xyz[2] - A1[2], 2)) - d1) 
                  //+ 2 * cov_inv(3, 5) * (ceres::sqrt(ceres::pow(xyz[0] - A0[0], 2) + ceres::pow(xyz[1] - A0[1], 2) + ceres::pow(xyz[2] - A0[2], 2)) - d0) * 
                  //                     (ceres::sqrt(ceres::pow(xyz[0] - A2[0], 2) + ceres::pow(xyz[1] - A2[1], 2) + ceres::pow(xyz[2] - A2[2], 2)) - d2) 
                  //+ 2 * cov_inv(4, 5) * (ceres::sqrt(ceres::pow(xyz[0] - A1[0], 2) + ceres::pow(xyz[1] - A1[1], 2) + ceres::pow(xyz[2] - A1[2], 2)) - d1) * 
                  //                     (ceres::sqrt(ceres::pow(xyz[0] - A2[0], 2) + ceres::pow(xyz[1] - A2[1], 2) + ceres::pow(xyz[2] - A2[2], 2)) - d2) 
                  ) ; 
    return true;
  }
  const double xv, yv, zv, d0, d1, d2;
  const MatrixXd cov_inv;
};

//数据划窗，用于协方差计算
void slide_window(vector<double> &Data, int &window_size, queue<vector<double>> &window){
    if(window.size() == window_size){
      window.pop();
    }
    window.push(Data);

}
//协方差计算
vector<vector<double>> covariance_matrix(vector<vector<double>> data) {
    int n = data.size(); // 数据集的大小
    int m = data[0].size(); // 每个数据点的维度
    vector<vector<double>> cov(m, vector<double>(m, 0.0)); // 初始化协方差矩阵为0

    // 计算每一对变量之间的协方差
    for (int i = 0; i < m; i++) {
        for (int j = 0; j <= i; j++) {
            vector<double> Xi, Xj;
            for (int k = 0; k < n; k++) {
                Xi.push_back(data[k][i]);
                Xj.push_back(data[k][j]);
            }
            double sumXi = accumulate(Xi.begin(), Xi.end(), 0.0);
            double sumXj = accumulate(Xj.begin(), Xj.end(), 0.0);
            double meanXi = sumXi / n;
            double meanXj = sumXj / n;

            vector<double> XY;
            for (int k = 0; k < n; k++) {
                XY.push_back((data[k][i] - meanXi) * (data[k][j] - meanXj));
            }
            double sumXY = accumulate(XY.begin(), XY.end(), 0.0);
            cov[i][j] = cov[j][i] = sumXY / n; // 因为是对称矩阵，所以同时更新cov[i][j]和cov[j][i]
        }
    }

    return cov;
}


void GetVIO(const nav_msgs::Odometry& VIO){
  x_VIO = VIO.pose.pose.position.x;
  y_VIO = VIO.pose.pose.position.y;
  z_VIO = VIO.pose.pose.position.z;
  //ROS_INFO("###Get VIO!:[x: %f m, y: %f m, T0-A2: %f m]", x_VIO, y_VIO, z_VIO);
  xyz[0] = x_VIO;
  xyz[1] = y_VIO;
  xyz[2] = z_VIO;
  vector<double> P = {x_VIO, y_VIO, z_VIO};
  slide_window(P, window_size_VIO, VIO_window);
}

void GetUWB_dis(const geometry_msgs::Vector3& uwb_dis){
   T0_A0 = double(uwb_dis.x);
   T0_A1 = double(uwb_dis.y);
   T0_A2 = double(uwb_dis.z);
   //ROS_INFO("Get UWB!###:[T0-A0: %f m, T0-A1: %f m, T0-A2: %f m]", T0_A0, T0_A1, T0_A2);
   cur_data[0] = x_VIO;
   cur_data[1] = y_VIO;
   cur_data[2] = z_VIO;
   cur_data[3] = T0_A0;
   cur_data[4] = T0_A1;
   cur_data[5] = T0_A2;
   
   slide_window(cur_data, window_size_cor, cor_window);
   flag ++ ;
   if(flag == 5){
    cor_vector.clear();
    queue<vector<double>> window = cor_window; 
    for(int i = 0; i < cor_window.size(); i ++){
      cor_vector.push_back(window.front());
      window.pop();
        
    }
    cov = covariance_matrix(cor_vector);
    for(int x = 0; x < 6; x ++){
      for(int y = 0; y < 6; y++){
        cov_Matrix(x, y) = cov[x][y];
      }
    }
    cov_ivs = cov_Matrix.inverse();
    flag = 0;
   }
   
   ceres::Problem problem;
    
   problem.AddResidualBlock(     // 向问题中添加误差项
      // 使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致
      new ceres::AutoDiffCostFunction<FITTING_COST, 1, 3>(
        //new FITTING_COST(x_VIO, y_VIO, z_VIO, T0_A0, T0_A1, T0_A2, cov_ivs)  //用当前VIO数据进行优化
        new FITTING_COST(VIO_window.front()[0], VIO_window.front()[1], VIO_window.front()[2], T0_A0, T0_A1, T0_A2, cov_ivs) // 用距离当前帧最近的VIO帧进行优化
      ),
      new ceres::HuberLoss(0.5),//鲁棒核函数
      xyz                 // 待估计参数
    );
  
  //chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
  ceres::Solve(options, &problem, &summary);  // 开始优化
  //chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  //chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
  //cout << "solve time cost = " << time_used.count() << " seconds. " << endl;

  // 输出结果
  //cout << summary.BriefReport() << endl;
  //cout << "estimated x,y,z = ";
  //for (auto a:xyz) cout << a << " ";
  //cout << endl;
  result_msgs.header.stamp = ros::Time::now();
  result_msgs.pose.position.x = xyz[0];
  result_msgs.pose.position.y = xyz[1];
  result_msgs.pose.position.z = xyz[2];
  
  optmz_result.publish(result_msgs);
  
  //ROS_INFO("Publish Position[x: %f m, y: %f m, z: %f m]", result_msgs.point.x, result_msgs.point.y, result_msgs.point.z);

}



int main(int argc, char **argv) {
  
  options.linear_solver_type = ceres::DENSE_SCHUR;  // 增量方程如何求解
  options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  options.minimizer_progress_to_stdout = true;   // 输出到cout
  //options.max_num_iterations = 20;

  ros::init(argc, argv, "UWB");
  ros::NodeHandle n;
  optmz_result = n.advertise<geometry_msgs::PoseStamped>("/UWB_module/optimization_result", 10);
  ros::Subscriber VIO_sub = n.subscribe("/vins_fusion/imu_propagate", 10, GetVIO);
  ros::Subscriber UWB_sub = n.subscribe("/UWB_module/distance", 10, GetUWB_dis);
  ros::spin();

}