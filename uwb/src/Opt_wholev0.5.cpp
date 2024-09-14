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

// 初始化变量和常量
Eigen::Vector3d x(0.0, 0.0, 0.0);
Eigen::Quaterniond q(1.0, 0.0, 0.0, 0.0);
Eigen::Vector3d P_VIO(0.0, 0.0, 0.0);
Eigen::Vector3d A0(5.7, -0.08, 1.735);
Eigen::Vector3d UWB_IMU(0.0, -0.08, 0.1);
vector<double> cur_data(6);
double T0_A0, T0_A1, T0_A2;
queue<Eigen::Vector3d> VIO_window;
int VIO_winsize = 3;


geometry_msgs::PoseStamped result_msgs;
//ros::NodeHandle n;
ros::Publisher optmz_result;




ceres::Solver::Options options;     // 配置项
ceres::Solver::Summary summary;



struct CostFunctor {
    CostFunctor(const Eigen::Vector3d& x1, const Eigen::Vector3d& X,
                const Eigen::Vector3d& x2, const double d)
        : x1_(x1), X_(X), x2_(x2), d_(d) {}

    template<typename T>
    bool operator()(const T* const x, const T* const q, T* residuals) const {
        // 定义常量和变量
        const Eigen::Matrix<T, 3, 1> x1 = x1_.cast<T>();
        const Eigen::Matrix<T, 3, 1> X = X_.cast<T>();
        const Eigen::Matrix<T, 3, 1> x2 = x2_.cast<T>();
        const T d = static_cast<T>(d_);

        Eigen::Map<const Eigen::Matrix<T, 3, 1>> x_map(x);
        Eigen::Map<const Eigen::Quaternion<T>> q_map(q);

        // 计算旋转后的向量
        Eigen::Matrix<T, 3, 1> x2_rotated = q_map * x2 + x_map - X;

        // 计算残差
        residuals[0] = (x1 - x_map).norm() + (x2_rotated.norm() - d) * (x2_rotated.norm() - d);
        //residuals[0] = (x1 - x_map).norm() + ((q_map * x2 + x_map - X).norm() - d) * ((q_map * x2 + x_map - X).norm() - d);

        return true;
    }

    Eigen::Vector3d x1_;
    Eigen::Vector3d X_;
    Eigen::Vector3d x2_;
    double d_;
};

//数据划窗，用于协方差计算
void slide_window(Eigen::Vector3d &Data, int &window_size, queue<Eigen::Vector3d> &window){
    if(window.size() == window_size){
      window.pop();
    }
    window.push(Data);

}
void slide_window_forCov(vector<double> &Data, int &window_size, queue<vector<double>> &window){
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
  P_VIO[0] = VIO.pose.pose.position.x;
  P_VIO[1] = VIO.pose.pose.position.y;
  P_VIO[2] = VIO.pose.pose.position.z;
  q.w() = VIO.pose.pose.orientation.w;
  q.x() = VIO.pose.pose.orientation.x;
  q.y() = VIO.pose.pose.orientation.y;
  q.z() = VIO.pose.pose.orientation.z;
  //ROS_INFO("###Get VIO!:[x: %f m, y: %f m, T0-A2: %f m]", x_VIO, y_VIO, z_VIO);
//   x[0] = P_VIO[0];
//   x[1] = P_VIO[1];
//   x[2] = P_VIO[2];
   slide_window(P_VIO, VIO_winsize, VIO_window);
  //vector<double> P = {x_VIO, y_VIO, z_VIO};
  //slide_window(P, window_size_VIO, VIO_window);
}

void GetUWB_dis(const geometry_msgs::Vector3& uwb_dis){
   T0_A0 = double(uwb_dis.x);
   T0_A1 = double(uwb_dis.y);
   T0_A2 = double(uwb_dis.z);
   //ROS_INFO("Get UWB!###:[T0-A0: %f m, T0-A1: %f m, T0-A2: %f m]", T0_A0, T0_A1, T0_A2);
   x[0] = VIO_window.front()[0];
   x[1] = VIO_window.front()[1];
   x[2] = VIO_window.front()[2];

   
   ceres::Problem problem;
   problem.AddParameterBlock(x.data(), 3);
   problem.AddParameterBlock(q.coeffs().data(), 4, new ceres::EigenQuaternionParameterization);
    
   // 添加残差项
    ceres::CostFunction* cost_function =
        new ceres::AutoDiffCostFunction<CostFunctor, 2, 3, 4>(new CostFunctor(P_VIO, A0, UWB_IMU, T0_A0));
    problem.AddResidualBlock(cost_function, 
                             new ceres::HuberLoss(0.5),//鲁棒核函数
                             x.data(), q.coeffs().data());

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
  result_msgs.pose.position.x = x[0];
  result_msgs.pose.position.y = x[1];
  result_msgs.pose.position.z = x[2];
  result_msgs.pose.orientation.w = q.w();
  result_msgs.pose.orientation.x = q.x();
  result_msgs.pose.orientation.y = q.y();
  result_msgs.pose.orientation.z = q.z();
  
  optmz_result.publish(result_msgs);
  
  //ROS_INFO("Publish Position[x: %f m, y: %f m, z: %f m]", result_msgs.point.x, result_msgs.point.y, result_msgs.point.z);

}



int main(int argc, char **argv) {
  
  options.linear_solver_type = ceres::DENSE_SCHUR;  // 增量方程如何求解
  options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  options.minimizer_progress_to_stdout = true;   // 输出到cout
  //options.max_num_iterations = 20;

  ros::init(argc, argv, "Optimization");
  ros::NodeHandle n;
  optmz_result = n.advertise<geometry_msgs::PoseStamped>("/UWB_module/optimization_result", 10);
  ros::Subscriber VIO_sub = n.subscribe("/vins_fusion/imu_propagate", 10, GetVIO);
  ros::Subscriber UWB_sub = n.subscribe("/UWB_module/distance", 10, GetUWB_dis);
  ros::spin();

}