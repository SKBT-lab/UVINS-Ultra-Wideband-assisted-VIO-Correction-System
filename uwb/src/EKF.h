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

using namespace Eigen;
using std::vector;


class E_K_FILTER{
private:
    Matrix3d H; //Jarcobin矩阵
    Matrix3d A;
    Matrix3d B;
    Matrix3d Q;
    Matrix3d R;
    Matrix3d K;
    Matrix3d P_pre;
    Vector3d x_p_pre;
    double T0_A0, T0_A1, T0_A2;
    Vector3d T_A;
    Vector3d A0;
    Vector3d A1;
    Vector3d A2;
    vector<Vector3d> Anchor;
    

    

public:
    Vector3d x_pv;
    Vector3d x_p;
    Matrix3d P;
    void Compute_jacobian();
    void Input_UWB(const double& d1, const double& d2, const double& d3);
    void ekf_predict(const Vector3d& u);
    Vector3d observationModel(const Vector3d& x);
    void ekf_update();
        E_K_FILTER(){
        A0 << -0.02, 1.33, 0.86;//26.7
        A1 << -0.021, -3.05, 0.578;
        A2 << 1.215, 1.815, 0.82;
        Anchor.push_back(A0);
        Anchor.push_back(A1);
        Anchor.push_back(A2);
        x_p << 0.0, 0.0, 0.0;
        x_p_pre << 0.0, 0.0, 0.0;
        x_pv << 0.0, 0.0, 0.0;
        A.setIdentity();
        P.setIdentity();
        B.setZero();
        Q = 0.2 * A;
        R = 0.2 * A;
    }
    
    


};

