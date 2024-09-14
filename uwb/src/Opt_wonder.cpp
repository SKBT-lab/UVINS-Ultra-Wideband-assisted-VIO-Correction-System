#include <iostream>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
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



 // 初始化变量和常量
Eigen::Vector3d x(0.0, 0.0, 0.0);
Eigen::Quaterniond q(1.0, 0.0, 0.0, 0.0);
Eigen::Vector3d x1(1.0, 2.0, 0.0);
Eigen::Vector3d X(4.0, 5.0, 6.0);
Eigen::Vector3d x2(7.0, 8.0, 9.0);

// 定义CostFunctor
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

        return true;
    }

    Eigen::Vector3d x1_;
    Eigen::Vector3d X_;
    Eigen::Vector3d x2_;
    double d_;
};

int main(int argc, char** argv) {
    // 初始化问题
    ceres::Problem problem;

   
    double d = 10.0;

    // 添加优化变量
    problem.AddParameterBlock(x.data(), 3);
    problem.AddParameterBlock(q.coeffs().data(), 4, new ceres::EigenQuaternionParameterization);

    // 添加残差项
    ceres::CostFunction* cost_function =
        new ceres::AutoDiffCostFunction<CostFunctor, 1, 3, 4>(new CostFunctor(x1, X, x2, d));
    problem.AddResidualBlock(cost_function, nullptr, x.data(), q.coeffs().data());

    // 设置优化器选项并运行
    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // 输出结果
    std::cout << summary.FullReport() << std::endl;
    std::cout << "x = " << x.transpose() << std::endl;
    std::