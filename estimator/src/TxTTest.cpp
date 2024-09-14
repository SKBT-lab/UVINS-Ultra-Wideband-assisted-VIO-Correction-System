#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include "eliminator/UWB_eliminator.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>

class SlidingWindow {
public:
    SlidingWindow(size_t windowSize) : maxSize(windowSize) {}

    // 向滑动窗口中添加新的Eigen::Vector3d数据
    void addData(const Eigen::Vector3d& newData) {
        if (window.size() >= maxSize) {
            window.pop_front();  // 移除最早的数据
        }
        window.push_back(newData);  // 添加最新的数据
    }

    // 获取滑动窗口中的所有数据
    std::deque<Eigen::Vector3d> getWindowData() const {
        return window;
    }

    // 获取滑动窗口的大小
    size_t getWindowSize() const {
        return window.size();
    }

    // 返回滑动窗口中所有Eigen::Vector3d之和
    Eigen::Vector3d getSum() const {
        Eigen::Vector3d sum = Eigen::Vector3d::Zero();  // 初始化为零向量
        for (const auto& vec : window) {
            sum += vec;  // 累加每个向量
        }
        return sum;
    }

private:
    std::deque<Eigen::Vector3d> window;  // 用双端队列存储滑动窗口中的数据
    size_t maxSize;  // 滑动窗口的最大大小
};

int main() {
    std::string filename = "/home/skbt/VIU_ROSBAG/orbslam/merged.txt";  // 文件名
    std::string output_filename = "/home/skbt/VIU_ROSBAG/orbslam/orb+uwb.txt";  // 输出文件名
    std::string output_filename2 = "/home/skbt/VIU_ROSBAG/orbslam/traxyz.txt";  // 输出文件名

    std::ifstream infile(filename);  // 打开文件
    std::ofstream outfile(output_filename); // 打开输出文件
    std::ofstream outfile2(output_filename2); // 打开输出文件

    E_K_FILTER elimanator;

    SlidingWindow slidingWindow(5);


    // 检查文件是否成功打开
    if (!infile.is_open()) {
        std::cerr << "Error: Cannot open file " << filename << std::endl;
        return 1;
    }

    std::string line;
    // 逐行读取文件
    while (std::getline(infile, line)) {
        std::istringstream iss(line);  // 将一行数据放入字符串流中

        double timestamp;              // 存储时间戳
        iss >> timestamp;              // 读取时间戳

        std::vector<double> data;      // 存储该行的数据
        double value;
        // 逐个读取行中的其他数据
        while (iss >> value) {
            data.push_back(value);     // 将数据加入到向量中
        }

        Eigen::Vector3d P(data[0], data[1], data[2]);
        Eigen::Vector3d T_A(data[7], data[8], data[9]);
        Eigen::Quaterniond Q;
        Q.w() = data[6];
        Q.x() = data[3];
        Q.y() = data[4];
        Q.z() = data[5];
        Eigen::Matrix3d rm_b_c;
        rm_b_c << 0.0, 0.0, 1.0,
                -1.0, 0.0, 0.0,
                0.0, -1.0, 0.0;
        // Eigen::Quaterniond q_b_s = Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitY());
        P = rm_b_c * P;
        Q = rm_b_c * Q * rm_b_c.inverse();
        outfile2 << std::fixed << std::setprecision(6) << timestamp << " " << P[0] << " " << P[1] << " " << P[2] << std::endl;
        P = P + slidingWindow.getSum();
            
        T_A = elimanator.Mean_filter(T_A);
        //Vector3d Dis = elimanator.Kalman_filter(dis_);
        //Vector3d Dis = dis_;
        // Q.w() = 1;
        // Q.x() = 0;
        // Q.y() = 0;
        // Q.z() = 0;
        elimanator.Input_UWB(timestamp, T_A);
        elimanator.Input_VIO(timestamp, P, Q);
        elimanator.processUWB();
        elimanator.Initialization();
        elimanator.Update_state();
        elimanator.Optimization();
        std::cout << "dP: " << elimanator.correction << std::endl;
        // if(elimanator.correction.norm() < 0.2){
        //    P = P + elimanator.correction;    
        // }
        
        P = P + elimanator.correction; 
        slidingWindow.addData(elimanator.correction);

        
        

        
        // std::cout << "Timestamp: " << timestamp << " | Pos: " << P[0] << ", " << P[1] << ", " << P[2] << std::endl;
        outfile << std::fixed << std::setprecision(6) << timestamp << " " << P[0] << " " << P[1] << " " << P[2] << std::endl;  // 写入时间戳
    }

    return 0;
}