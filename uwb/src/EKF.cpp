#include "EKF.h"

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

    std::cout << "Jacobian_Matrix:\n" << H << std::endl;

}

void E_K_FILTER::Input_UWB(const double& d1, const double& d2, const double& d3){
    T0_A0 = d1;
    T0_A1 = d2;
    T0_A2 = d2;
    T_A << d1, d2, d3;
}


void E_K_FILTER::ekf_predict(const Vector3d& u){
  x_p_pre = A * x_p + B * u;
  P_pre = A * P * A.transpose() + Q;
}

void E_K_FILTER::ekf_update(){
  Vector3d y = T_A - observationModel(x_p_pre);
  Compute_jacobian();
  Matrix3d S = H * P_pre * H.transpose() + R;
  K = P_pre * H.transpose() * S.inverse();
  x_p = x_p_pre + K * y;
  P = P_pre - K * H * P_pre;
}
