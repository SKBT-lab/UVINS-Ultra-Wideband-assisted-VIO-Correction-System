#include "UWB_eliminator.h"
#include <cmath>

struct UWBErr {
    UWBErr(const Vector3d& x1, const Vector3d& X0, const Vector3d& X1, const Vector3d& X2,
                const Vector3d& U_I, const Vector3d& t_a , const Quaterniond& q, const MatrixXd& h)
        : x1_(x1), X0_(X0), X1_(X1), X2_(X2), U_I_(U_I), t_a_(t_a), q_(q),H_(h) {} //VIO、三个基站坐标、UWB与IMU之间的外参、UWB测距值、姿态、协方差
   
    template<typename T>
    bool operator()(const T* const x, T* residuals) const {

        Map<const Matrix<T, 3, 1>> x_err(x); //待优化变量

        // 测距误差向量
        Matrix<T, 3, 1> T0A0 = q_ * U_I_ + x_err + x1_ - X0_;
        Matrix<T, 3, 1> T0A1 = q_ * U_I_ + x_err + x1_ - X1_;
        Matrix<T, 3, 1> T0A2 = q_ * U_I_ + x_err + x1_ - X2_;
        Matrix<T, 3, 1> Dis_Vec(T(T0A0.norm()), T(T0A1.norm()), T(T0A2.norm()));
        //Eigen::Matrix<T, 3, 1> Dis_Vec(T(T0A0.norm()), T(0.0), T(0.0));
        Matrix<T, 3, 1> Dis_err = Dis_Vec - t_a_;
        // Matrix<T, 6, 6> H;
        // H.setZero();
        // H.template block<3, 3>(0, 0) = cor_VIO.template block<3, 3>(0, 0);
        // H.template block<3, 3>(3, 3) = cor_UWB.template block<3, 3>(0, 0);

        // 计算残差
        //residuals[0] = (x1_ - x_map).norm() + (Dis_Vec - t_a_).norm();
        residuals[0] = Dis_err.transpose() * H_ * Dis_err;

        //residuals[0] = P_err.transpose() * cor_VIO * P_err + Dis_err.transpose() * cor_UWB * Dis_err ;
        //residuals[0] = (x1 - x_map).norm() + ((q_map * x2 + x_map - X).norm() - d) * ((q_map * x2 + x_map - X).norm() - d);

        return true;
    }

    Vector3d x1_;
    Vector3d U_I_;
    Vector3d X0_;
    Vector3d X1_;
    Vector3d X2_;
    Vector3d t_a_;
    Quaterniond q_;
    MatrixXd H_;
};
struct VIOErr {
    VIOErr(const MatrixXd& h): H_(h){} //VIO、三个基站坐标、UWB与IMU之间的外参、UWB测距值、姿态、协方差

    template<typename T>
    bool operator()(const T* const x, T* residuals) const {

        Map<const Matrix<T, 3, 1>> x_err(x); //待优化变量
        Matrix<T, 3, 1> P_err = x_err;
    
        // 计算残差
        //residuals[0] = (x1_ - x_map).norm() + (Dis_Vec - t_a_).norm();
        residuals[0] = P_err.transpose() * H_ * P_err;

        //residuals[0] = P_err.transpose() * cor_VIO * P_err + Dis_err.transpose() * cor_UWB * Dis_err ;
        //residuals[0] = (x1 - x_map).norm() + ((q_map * x2 + x_map - X).norm() - d) * ((q_map * x2 + x_map - X).norm() - d);

        return true;
}
    MatrixXd H_;   
};

struct SmoothErr {
    SmoothErr(const MatrixXd& h, const Vector3d& p_VIO, const Vector3d& p_VIO_pre): H_(h), p_VIO_(p_VIO), p_VIO_pre_(p_VIO_pre){} //VIO、三个基站坐标、UWB与IMU之间的外参、UWB测距值、姿态、协方差

    template<typename T>
    bool operator()(const T* const x, const T* const x_pre,T* residuals) const {

        Map<const Matrix<T, 3, 1>> dp(x); //待优化变量
        Map<const Matrix<T, 3, 1>> dp_pre(x_pre);
        Matrix<T, 3, 1> Smooth_err = p_VIO_ + dp - p_VIO_pre_ - dp_pre;
        residuals[0] = Smooth_err.transpose() * H_ * Smooth_err;
        return true;
}

  
    MatrixXd H_;
    Vector3d p_VIO_;
    Vector3d p_VIO_pre_;
    
    
};


Vector3d E_K_FILTER::Mean_filter(const Vector3d& d){
    Mean_window.push(d);
    Window_sum += d;

    if (Mean_window.size() > Mean_window_size){
        Window_sum -= Mean_window.front();
        Mean_window.pop();
    }
    return Window_sum / Mean_window.size();
}

Vector3d E_K_FILTER::Kalman_filter(const Vector3d& measurement) {
        
        if (state_.isZero()){
            state_ = measurement;
        }
        else{
        // 预测步骤
        Vector3d predicted_state = state_;
        Matrix3d predicted_state_covariance = state_covariance_ + process_noise_covariance_;

        // 更新步骤
        Matrix3d kalman_gain = predicted_state_covariance * (predicted_state_covariance + measurement_noise_covariance_).inverse();
        state_ = predicted_state + kalman_gain * (measurement - predicted_state);
        state_covariance_ = (Matrix3d::Identity() - kalman_gain) * predicted_state_covariance;
        }
        return state_;
    }

Vector3d E_K_FILTER::observationModel(const Vector3d& x) {
    // Assuming plane_coordinate is a global variable or passed as an argument
    // plane_coordinate[i][0], plane_coordinate[i][1], plane_coordinate[i][2] corresponds to the coordinates of the ith landmark

    Vector3d observations;
    for (int i = 0; i < 3; ++i) {
        // Calculate the distance between the state and each landmark.
        double distance = (Anchor[i][0] - x[0]) * (Anchor[i][0] - x[0]) +
                          (Anchor[i][1] - x[1]) * (Anchor[i][1] - x[1]) +
                          (Anchor[i][2] - x[2]) * (Anchor[i][2] - x[2]);

        distance = std::sqrt(distance);
        observations(i) = distance;
    }

    return observations;
}

Vector3d E_K_FILTER::fitParabola(const Vector2d& p1, const Vector2d& p2, const Vector2d& p3){
    Vector3d para;
    double denom = (p1[0] - p2[0]) * (p1[0] - p3[0]) * (p2[0] - p3[0]);
    para[0] = (p3[0] * (p2[1] - p1[1]) + p2[0] * (p1[1] - p3[1]) + p1[0] * (p3[1] - p2[1])) / denom;
    para[1] = (p3[0] * p3[0] * (p1[1] - p2[1]) + p2[0] * p2[0] * (p3[1] - p1[1]) + p1[0] * p1[0] * (p2[1] - p3[1])) / denom;
    para[2] = (p2[0] * p3[0] * (p2[0] - p3[0]) * p1[1] + p3[0] * p1[0] * (p3[0] - p1[0]) * p2[1] + p1[0] * p2[0] * (p1[0] - p2[0]) * p3[1]) / denom;
    return para;
}

double E_K_FILTER::interpolate(const std::vector<Vector2d>& D, double t) {
    Vector4d para;
    
    Align_matrix << D[0][0] * D[0][0] * D[0][0], D[0][0] * D[0][0], D[0][0], 1,
                    D[1][0] * D[1][0] * D[1][0], D[1][0] * D[1][0], D[1][0], 1,
                    D[2][0] * D[2][0] * D[2][0], D[2][0] * D[2][0], D[2][0], 1,
                    D[3][0] * D[3][0] * D[3][0], D[3][0] * D[3][0], D[3][0], 1;
    Align_vector << D[0][1], D[1][1], D[2][1], D[3][1];
    para = Align_matrix.inverse() * Align_vector;
    // dx = 3 * para[0] * D[3][0] * D[3][0] + 2 * para[1] * D[3][0] + para[2];
    // ddx = 6 * para[0] * D[3][0] + 2 * para[1];
    
    // else{
    //     Align_matrix << D[2][0] * D[2][0] * D[2][0], D[2][0] * D[2][0], D[2][0], 1,
    //                     D[3][0] * D[3][0] * D[3][0], D[3][0] * D[3][0], D[3][0], 1,
    //                     3 * D[2][0] * D[2][0]      , 2 * D[2][0]      , 1      , 0,
    //                     6 * D[2][0]                , 2                , 0      , 0;
    //     Align_vector << D[2][1], D[3][1], dx, ddx;
    //     para = Align_matrix.inverse() * Align_vector;
    //     dx = 3 * para[0] * D[3][0] * D[3][0] + 2 * para[1] * D[3][0] + para[2];
    //     ddx = 6 * para[0] * D[3][0] + 2 * para[1];
    // }
    return para[0] * t * t * t + para[1] * t * t + para[2] * t + para[3];
}

void E_K_FILTER::Compute_jacobian() {

    double dx0 = x_p_pre[0] - A0[0];
    double dy0 = x_p_pre[1] - A0[1];
    double dz0 = x_p_pre[2] - A0[2];
    double dx1 = x_p_pre[0] - A1[0];
    double dy1 = x_p_pre[1] - A1[1];
    double dz1 = x_p_pre[2] - A1[2];
    double dx2 = x_p_pre[0] - A2[0];
    double dy2 = x_p_pre[1] - A2[1];
    double dz2 = x_p_pre[2] - A2[2];

    Vector3d t_a = observationModel(x_p_pre);

    H << dx0 / t_a[0], dy0 / t_a[0], dz0 / t_a[0],
         dx1 / t_a[1], dy1 / t_a[1], dz1 / t_a[1],
         dx2 / t_a[2], dy2 / t_a[2], dz2 / t_a[2];

    //std::cout << "Jacobian_Matrix:\n" << H << std::endl;

}

void E_K_FILTER::Input_UWB(const double& t, const Vector3d& dis){
    if(Firstframe_Flag == true){
        First_time = t;
        Firstframe_Flag = false;
    }
    std::pair<double, Vector3d> uwb_element = std::make_pair(t - First_time, dis);
    // outfile1 << std::fixed << std::setprecision(6)<< t << " " 
    //         << dis[0] << " " 
    //         << dis[1] << " " 
    //         << dis[2] << std::endl;
    uwbBuf.push(uwb_element);
    if(uwbBuf.size() > 4){
        uwbBuf.pop();
    }
}
bool E_K_FILTER::uwb_available(Vector3d D){
    if(D[0] > 0.2 && D[1] > 0.2 && D[2] > 0.2)
        return true;
    else
        return false;
}
void E_K_FILTER::Input_VIO(const double& t, const Vector3d& p, const Quaterniond& q){
    if(Firstframe_Flag == true){
        First_time = t;
        Firstframe_Flag = false;
    }
    t_VIO = t - First_time;
    x_p_VIO = p;
    x_q = q;
}

void E_K_FILTER::ekf_predict(){
  x_p_pre = x_p_VIO;
  P_pre = A * P * A.transpose() + Q;
}

void E_K_FILTER::ekf_update(){
  Vector3d y = T_A - observationModel(x_p_pre);
  Compute_jacobian();
  //std::cout << "H\n" << H << std::endl;
  Matrix3d S = H * P_pre * H.transpose() + R;
  K = P_pre * H.transpose() * S.inverse();
  x_p = x_p_pre + K * y;
  Init_correction = K * y; //后续优化初值
  Init_correction << 0.0, 0.0, 0.0;
  P = P_pre - K * H * P_pre;
}
void E_K_FILTER::processUWB(){
    if(uwbBuf.size() == 4){
        std::vector<std::pair<double, Vector3d>> uwbVec;
        for (size_t i = 0; i < 4; ++i) {
        uwbVec.push_back(uwbBuf.front());
        uwbBuf.push(uwbBuf.front());
        uwbBuf.pop();
    }
        std::vector<Vector2d> D1;
        std::vector<Vector2d> D2;
        std::vector<Vector2d> D3;
        for (int i = 0; i < 4; ++i){
            // cout << i << " " << uwbVec[i].first << std::endl;
            Vector2d d1(uwbVec[i].first, uwbVec[i].second[0]);
            Vector2d d2(uwbVec[i].first, uwbVec[i].second[1]);
            Vector2d d3(uwbVec[i].first, uwbVec[i].second[2]);
            D1.push_back(d1);
            D2.push_back(d2);
            D3.push_back(d3);

        }
        // cout << "T_VIO: " << t_VIO;
        T0_A0 = interpolate(D1, t_VIO);
        T0_A1 = interpolate(D2, t_VIO);
        T0_A2 = interpolate(D3, t_VIO);
        T_A << T0_A0, T0_A1, T0_A2;
        //std::cout << "T_A:"<< T0_A0<< " " << T0_A1<< " " << T0_A2 << std::endl;

        // //存储基于Smoothness Driven Ransac的二次样条插值时间戳对齐结果
        // outfile << std::fixed << std::setprecision(6)<< t_VIO + First_time << " " 
        //     << T0_A0 << " " 
        //     << T0_A1 << " " 
        //     << T0_A2 << std::endl;
        

        // //存储一次样条插值时间戳对齐结果
        // double t1 = uwbVec[2].first;
        // Vector3d Dis1 = uwbVec[2].second;
        // double t2 = uwbVec[3].first;
        // Vector3d Dis2 = uwbVec[3].second;
        // Vector3d dis;
        // dis = Dis1 + (Dis2 - Dis1) * (t_VIO - t1) / (t2 - t1);
        // outfile1 << std::fixed << std::setprecision(6)<< t_VIO + First_time << " " 
        //     << dis[0] << " " 
        //     << dis[1] << " " 
        //     << dis[2] << std::endl;
        
        // //存储二次样条插值时间戳对齐结果
        // Vector3d para1;
        // Vector3d para2;
        // Vector3d para3;
        // para1 = fitParabola(D1[1], D1[2], D1[3]);
        // para2 = fitParabola(D2[1], D2[2], D2[3]);
        // para3 = fitParabola(D3[1], D3[2], D3[3]);
        // double dis1 = para1[0] * t_VIO * t_VIO +  para1[1] * t_VIO + para1[2];
        // double dis2 = para2[0] * t_VIO * t_VIO +  para2[1] * t_VIO + para2[2];
        // double dis3 = para3[0] * t_VIO * t_VIO +  para3[1] * t_VIO + para3[2];
        // outfile2 << std::fixed << std::setprecision(6)<< t_VIO + First_time << " " 
        //     << dis1 << " " 
        //     << dis2 << " " 
        //     << dis3 << std::endl;

    }
}

void E_K_FILTER::Update_state(){
    int k = Opt_window_size - 1;
    for(; k >=0; k--){
        if(!Us[k].isZero()){
            break;
        }
    }
    if(k == Opt_window_size - 1){
        for (int i = 0; i < (Opt_window_size - 1); ++i) {
        Us[i] = Us[i + 1];
        dPs[i] = dPs[i + 1];
        Vps[i] = Vps[i + 1];
        Vqs[i] = Vqs[i + 1];
        Headers[i] = Headers[i + 1];
        Ps[i] = Ps[i + 1];
    }
        Us[Opt_window_size - 1] = T_A;
        dPs[Opt_window_size - 1] = Init_correction;
        Vps[Opt_window_size - 1] = x_p_VIO;
        Vqs[Opt_window_size - 1] = x_q;
        Headers[Opt_window_size - 1] = t_VIO;
        Ps[Opt_window_size - 1] = P;
    }
    else{
        Us[k + 1] = T_A;
        dPs[k + 1] = Init_correction;
        Vps[k + 1] = x_p_VIO;
        Vqs[k + 1] = x_q;
        Headers[k + 1] = t_VIO;
        Ps[k + 1] = P;//务必在执行Initialization()之后再执行
    }
    

}

void E_K_FILTER::Optimization(){
    ceres::Problem problem;
    ceres::Solver::Options options;     // 配置项
    ceres::Solver::Summary summary;
    options.linear_solver_type = ceres::DENSE_SCHUR;  // 增量方程如何求解
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    //options.minimizer_progress_to_stdout = true;   // 输出到cout
    problem.AddParameterBlock(correction.data(), 3);
    //problem.AddParameterBlock(q.coeffs().data(), 4, new ceres::EigenQuaternionParameterization);
    
 
    //h.setIdentity();
    //ROS_INFO_STREAM("H: \n" << h);
   // 添加残差项
   for(int i = 0; i < Opt_window_size; ++i){
        double* estimate = dPs[i].data();
        problem.AddParameterBlock(estimate, 3);

        P_VIO = Ps[i];
        P_UWB = H * H * P_VIO;

        ceres::CostFunction* UWB_costfunction =
        new ceres::AutoDiffCostFunction<UWBErr, 1, 3>(new UWBErr(Vps[i], A0, A1, A2, UWB_IMU, Us[i], Vqs[i], P_UWB.inverse()));
        problem.AddResidualBlock(UWB_costfunction,
                             new ceres::HuberLoss(0.15),//鲁棒核函数
                             estimate);
        
        ceres::CostFunction* VIO_costfunction =
        new ceres::AutoDiffCostFunction<VIOErr, 1, 3>(new VIOErr(P_VIO.inverse()));
        problem.AddResidualBlock(VIO_costfunction,
                             new ceres::HuberLoss(0.15),//鲁棒核函数
                             estimate);

        if(i > 0){
            Matrix3d P_VIO_pre = Ps[i - 1] * (Headers[i] - Headers[i - 1]) / 0.1;
            //std::cout << Headers[i] - Headers[i - 1] << std::endl;
            double* estimate_pre = dPs[i - 1].data();
            ceres::CostFunction* Smooth_costfunction =
            new ceres::AutoDiffCostFunction<SmoothErr, 1, 3, 3>(new SmoothErr(P_VIO_pre.inverse(), Vps[i], Vps[i - 1]));
            problem.AddResidualBlock(Smooth_costfunction,
                             new ceres::HuberLoss(0.15),//鲁棒核函数
                             estimate,
                             estimate_pre
                             );


        }
   }
   

  //chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    ceres::Solve(options, &problem, &summary);  // 开始优化
    correction = dPs[Opt_window_size - 1];
}


void E_K_FILTER::Initialization(){
    ekf_predict();
    ekf_update();
}



Vector3d E_K_FILTER::Get_dP(){
    Vector3d dP_sum(0.0, 0.0, 0.0);

    if(!dPBuf.empty()){
        int len = dPBuf.size();
        int i = 0;
        while (!dPBuf.empty())
        {
            i ++;
            if(i != len){
                dP_sum += dPBuf.front();
                dPBuf.pop();
                
                if(i == len - 1){
                    dP_sum = dP_sum * 0.4 / i;
                }
            }
            else if(len != 1){
                dP_sum += dPBuf.front() * 0.6; //进行一个简单的权重分配，最新帧分配0.6，其余平分0.4
                dPBuf.pop();
            }
            else{
                dP_sum += dPBuf.front();
                dPBuf.pop();
            }
        }
    }
        return dP_sum;
    
    

}
