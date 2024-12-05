#ifndef ARMOR_PROCESSOR__TRACKER_HPP_
#define ARMOR_PROCESSOR__TRACKER_HPP_

// Eigen
#include <Eigen/Eigen>

// STD
#include "KalmanFilter.hpp"
#include "KuhnMunkres.hpp"
#include "globalParam.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <cfloat>
#include <deque>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>

enum class ArmorsNum { NORMAL_4 = 4, BALANCE_2 = 2, OUTPOST_3 = 3 };

class Tracker{
public:
    Tracker(GlobalParam &gp);
    void track(std::vector<Armor> &armors_curr, Translator &ts, double dt);
    
private:

    GlobalParam *gp;
    double dt;
    std::vector<ExtendedKalmanFilter> ekf_list;
    std::vector<Eigen::VectorXd> z_vector_list;
    std::vector<int> lost_frame_count;
    std::vector<Armor> armors_pred;

    void refine_zVector(int ekf_id);
    void create_new_ekf(Armor &armor);
    // double cost_threshold;
    
    using VecVecFunc = std::function<Eigen::VectorXd(const Eigen::VectorXd &)>;
    using VecMatFunc = std::function<Eigen::MatrixXd(const Eigen::VectorXd &)>;
    using VoidMatFunc = std::function<Eigen::MatrixXd()>;

    // 保存了使用的EKF模型
    VecVecFunc f;           // State transition vector function
    VecVecFunc h;           // Observation nonlinear vector function
    VecMatFunc j_f;         // Jacobian of f()
    VecMatFunc j_h;         // Jacobian of h()
    VoidMatFunc u_q;        // Process noise covariance matrix
    VecMatFunc u_r;         // Measurement noise covariance matrix
    VecVecFunc nomolize_residual;  // Nomalize residual function
};
#endif // ARMOR_PROCESSOR__TRACKER_HPP_
