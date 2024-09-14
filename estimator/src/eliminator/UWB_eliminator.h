#include <iostream>
#include <chrono>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <queue>
#include <Eigen/Dense>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <ceres/ceres.h>
#include "../estimator/parameters.h"
#include <fstream>
#include "../utility/tic_toc.h"

using namespace Eigen;



class E_K_FILTER{
private:
    Matrix3d H; //Jarcobin矩阵
    Matrix3d A;
    Matrix3d B;
    Matrix3d Q;
    Matrix3d R;
    Matrix3d K;
    Matrix3d P_pre;
    Matrix3d I; //单位阵
    Vector3d x_p_pre;
    Vector3d A0;
    Vector3d A1;
    Vector3d A2;
    Vector3d UWB_IMU;
    Matrix<double, 9, 9> h;
    std::queue<std::pair<double, Vector3d>> uwbBuf;
    
    std::queue<Vector3d> Mean_window;
    int Mean_window_size;

    vector<Vector3d> Anchor;

    Vector3d state_;  // 状态估计
    Matrix3d state_covariance_;  // 状态估计协方差矩阵
    Matrix3d process_noise_covariance_;  // 过程噪声协方差矩阵
    Matrix3d measurement_noise_covariance_;  // 测量噪声协方差矩阵
    

    

public:
    Vector3d x_pv;
    Vector3d x_p;
    Matrix3d P; //EKF的协方差矩阵
    Matrix3d P_VIO;//VIO的协方差
    Matrix3d P_UWB;//UWB的协方差
    Vector3d x_p_VIO;
    Vector3d Init_correction;
    Vector3d correction;
    Vector3d correction_pre;
    Quaterniond x_q;
    double T0_A0, T0_A1, T0_A2;
    double t_VIO;
    Vector3d T_A;
    Vector3d Window_sum;
    std::queue<Vector3d> dPBuf;
    Vector3d       Us[Opt_window_size]; //UWB数据滑窗
    Vector3d       dPs[Opt_window_size]; //矫正量滑窗
    Vector3d       Vps[Opt_window_size]; //VIO_P滑窗
    Quaterniond    Vqs[Opt_window_size]; //VIO_q滑窗
    double         Headers[Opt_window_size]; //时间戳滑窗
    Matrix3d       Ps[Opt_window_size]; //EKF协方差滑窗
    std::ofstream outfile;
    std::ofstream outfile1;
    std::ofstream outfile2;

    Matrix4d Align_matrix;
    Vector4d Align_vector;
    // bool AlignInit_Flag;
    // double dx;
    // double ddx;
    
    bool Firstframe_Flag;
    double First_time;

    bool uwb_available(Vector3d D);
    void Compute_jacobian();
    void Input_UWB(const double& t, const Vector3d& dis);
    void Input_VIO(const double& t, const Vector3d& p, const Quaterniond& q);
    void ekf_predict();
    Vector3d observationModel(const Vector3d& x);
    void Optimization();
    void ekf_update();
    void processUWB();
    void Initialization();
    Vector3d Mean_filter(const Vector3d& d);
    Vector3d Kalman_filter(const Vector3d& measurement);
    Vector3d Get_dP();
    void Update_state();
    Vector3d fitParabola(const Vector2d& p1, const Vector2d& p2, const Vector2d& p3);
    double interpolate(const std::vector<Vector2d>& D, double t);
    
    
    E_K_FILTER(){
        outfile.open("/home/skbt/VIU_ROSBAG/txt_Data/T_A.txt", std::ios::out);
        outfile1.open("/home/skbt/VIU_ROSBAG/txt_Data/T_A_one.txt", std::ios::out);
        outfile2.open("/home/skbt/VIU_ROSBAG/txt_Data/T_A_two.txt", std::ios::out);
        Firstframe_Flag = true;
        // AlignInit_Flag = true;
        Mean_window_size = 4;
        Window_sum << 0.0, 0.0, 0.0;
        A0 << -3.507, 1.671, 1.847 - 0.193;//0.193
        A1 << 2.011, -1.948, 1.865 - 0.193;
        A2 << 2.228, 1.485, 1.503 - 0.193;
        Anchor.push_back(A0);
        Anchor.push_back(A1);
        Anchor.push_back(A2);
        x_p << 0.0, 0.0, 0.0;
        x_p_pre << 0.0, 0.0, 0.0;
        x_pv << 0.0, 0.0, 0.0;
        correction << 0.0, 0.0, 0.0;
        correction_pre << 0.0, 0.0, 0.0;
        A.setIdentity();
        P.setIdentity();
        P_VIO.setIdentity();
        I.setIdentity();
        B.setZero();
        h.setZero();
        Q = 0.2 * A;
        R = 0.4 * A;
        UWB_IMU << 0.0, -0.06, 0.1; 

        state_.setZero();  
        state_covariance_.setIdentity();  
        process_noise_covariance_ = 0.1 * A;  
        measurement_noise_covariance_ = 0.8 * A; 

        for (int i = 0; i < Opt_window_size; ++i) {
        Us[i].setZero();
        dPs[i].setZero();
        Vps[i].setZero();
        Vqs[i].setIdentity();
        Headers[i] = 0;
        Ps[i].setIdentity();

    }
    }

    ~E_K_FILTER(){
        outfile.close();

    
    }
    
};

