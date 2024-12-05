#include "Eigen/Eigen"
#include "Eigen/src/Core/Matrix.h"
// #include "fftw_omega.hpp"
#include "gaoning.hpp"
#include "globalParam.hpp"
#include "globalText.hpp"
#include "monitor.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/core/hal/interface.h"
#include <AimAuto.hpp>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <ostream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

// 相机到云台转轴的平移向量
#define VECTOR_X 0.015
#define VECTOR_Y 0
#define VECTOR_Z 0.095
#define DIM_ERROR_DEEP 1.0
#define V_ZOOM 1.0
#define VYAW_ZOOM 1.0

AimAuto::AimAuto(GlobalParam *gp) : dt(1e-3f), level_count{0, 0, 0}, restart_time(0.0), ArmorSpeed_array(), isBigArmor()
{
    // // 初始化一个白色的1000x1000图像矩阵
    // empty = cv::Mat(1000, 1000, CV_8UC3, cv::Scalar(255, 255, 255));


    // // 初始化检测器
    // det = initDetector(gp->color);

    // // 动态分配内存用于z_list
    // z_list = new double[this->z_len + 1];

    // // 获取参数配置
    // this->getParam();

    // // 初始化两个追踪器
    // tracker_0 = new Tracker(config0.max_match_distance, config0.max_match_yaw_diff);
    // tracker_1 = new Tracker(config1.max_match_distance, config1.max_match_yaw_diff);

    // // 定义状态转移函数
    // auto f = [this](const Eigen::VectorXd &x)
    // {
    //     // 更新状态：位置和速度
    //     Eigen::VectorXd x_new = x;
    //     x_new(0) += x(1) * dt; // 更新xc
    //     x_new(2) += x(3) * dt; // 更新yc
    //     x_new(4) += x(5) * dt; // 更新za
    //     x_new(6) += x(7) * dt; // 更新yaw
    //     return x_new;
    // };

    // // 状态转移函数的雅可比矩阵
    // auto j_f = [this](const Eigen::VectorXd &)
    // {
    //     Eigen::MatrixXd f(9, 9);
    //     // clang-format off
    //     f <<  1,   dt,  0,   0,   0,   0,   0,   0,   0,
    //           0,   1,   0,   0,   0,   0,   0,   0,   0,
    //           0,   0,   1,   dt,  0,   0,   0,   0,   0, 
    //           0,   0,   0,   1,   0,   0,   0,   0,   0,
    //           0,   0,   0,   0,   1,   dt,  0,   0,   0,
    //           0,   0,   0,   0,   0,   1,   0,   0,   0,
    //           0,   0,   0,   0,   0,   0,   1,   dt,  0,
    //           0,   0,   0,   0,   0,   0,   0,   1,   0,
    //           0,   0,   0,   0,   0,   0,   0,   0,   1;
    //     // clang-format on
    //     return f;
    // };

    // // 观测函数
    // auto h = [](const Eigen::VectorXd &x)
    // {
    //     Eigen::VectorXd z(4);
    //     double xc = x(0), yc = x(2), yaw = x(6), r = x(8);
    //     z(0) = xc - r * cos(yaw); // xa - 计算观测位置xa
    //     z(1) = yc - r * sin(yaw); // ya - 计算观测位置ya
    //     z(2) = x(4);              // za - 观测z坐标
    //     z(3) = x(6);              // yaw - 观测yaw角
    //     return z;
    // };

    // // 观测函数的雅可比矩阵
    // auto j_h = [](const Eigen::VectorXd &x)
    // {
    //     Eigen::MatrixXd h(4, 9);
    //     double yaw = x(6), r = x(8);
    //     // clang-format off
    //     //    xc   v_xc yc   v_yc za   v_za yaw         v_yaw r
    //     h <<  1,   0,   0,   0,   0,   0,   r*sin(yaw), 0,   -cos(yaw),
    //           0,   0,   1,   0,   0,   0,   -r*cos(yaw),0,   -sin(yaw),
    //           0,   0,   0,   0,   1,   0,   0,          0,   0,
    //           0,   0,   0,   0,   0,   0,   1,          0,   0;
    //     // clang-format on
    //     return h;
    // };

    // // 过程噪声协方差矩阵 u_q_0
    // auto u_q_0 = [this]()
    // {
    //     Eigen::MatrixXd q(9, 9);
    //     double t{dt}, x{config0.s2qxyz_}, y{config0.s2qyaw_}, r{config0.s2qr_};
    //     // 计算各种噪声参数
    //     double q_x_x{pow(t, 4) / 4 * x}, q_x_vx{pow(t, 3) / 2 * x}, q_vx_vx{pow(t, 2) * x};
    //     double q_y_y{pow(t, 4) / 4 * y}, q_y_vy{pow(t, 3) / 2 * y}, q_vy_vy{pow(t, 2) * y};
    //     double q_r{pow(t, 4) / 4 * r};
    //     // clang-format off
    //     q <<  q_x_x,  q_x_vx, 0,      0,      0,      0,      0,      0,      0,
    //           q_x_vx, q_vx_vx,0,      0,      0,      0,      0,      0,      0,
    //           0,      0,      q_x_x,  q_x_vx, 0,      0,      0,      0,      0,
    //           0,      0,      q_x_vx, q_vx_vx,0,      0,      0,      0,      0,
    //           0,      0,      0,      0,      q_x_x,  q_x_vx, 0,      0,      0,
    //           0,      0,      0,      0,      q_x_vx, q_vx_vx,0,      0,      0,
    //           0,      0,      0,      0,      0,      0,      q_y_y,  q_y_vy, 0,
    //           0,      0,      0,      0,      0,      0,      q_y_vy, q_vy_vy,0,
    //           0,      0,      0,      0,      0,      0,      0,      0,      q_r;
    //     // clang-format on
    //     return q;
    // };

    // // 观测噪声协方差矩阵 u_r_0
    // auto u_r_0 = [this](const Eigen::VectorXd &z)
    // {
    //     Eigen::DiagonalMatrix<double, 4> r;
    //     double x = config0.r_xyz_factor;
    //     r.diagonal() << abs(x * z[0]), abs(x * z[1]), abs(1e-7), config0.r_yaw; // 定义观测噪声
    //     return r;
    // };


    // // 初始化误差估计协方差矩阵P
    // Eigen::DiagonalMatrix<double, 9> p0;
    // p0.setIdentity(); // P0矩阵单位化

    // // 创建扩展卡尔曼滤波器的实例
    // tracker_0->ekf = ExtendedKalmanFilter{f, h, j_f, j_h, u_q_0, u_r_0, p0};

    // // 设置追踪器的阈值
    // tracker_0->tracking_thres = tracker_1->tracking_thres = 10;
    // tracker_0->lost_thres = tracker_1->lost_thres = 10;

    // 保存全局参数及其他初始化
    det = new Detector(*gp); // 初始化检测器
    this->gp = gp;
    this->tar_list.clear(); // 清空目标列表
    this->last_time_ = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()); // 获取当前时间
    this->_K_.resize(3, 3);
    this->_K_ << (float)gp->fx, 0, (float)gp->cx, 0, (float)gp->fy, (float)gp->cy, 0, 0, 1; // 设置相机矩阵
}
AimAuto::~AimAuto()
{
    free(this->z_list);
}
// void AimAuto::getParam()
// {
//     cv::FileStorage fs;
//     std::string path = std::filesystem::current_path();
//     path = path + "/config/AimautoConfig.yaml";
//     fs.open(path, cv::FileStorage::READ);
//     fs["max_match_distance"] >> config0.max_match_distance;
//     fs["max_match_yaw_diff"] >> config0.max_match_yaw_diff;
//     fs["s2qxyz"] >> config0.s2qxyz_;
//     fs["s2qyaw"] >> config0.s2qyaw_;
//     fs["s2qr"] >> config0.s2qr_;
//     fs["r_xyz_factor"] >> config0.r_xyz_factor;
//     fs["r_yaw"] >> config0.r_yaw;
// }
void AimAuto::AimAutoYHY(cv::Mat &src, Translator &ts)
{
    // ts.message.status = 2;
    this->is_hitting_outpose = ts.message.status % 5 == 2;
    this->tar_list.clear();
    auto armors = det->detect(src, gp->color);
    std::sort(armors.begin(), armors.end(), [&](const UnsolvedArmor &la, const UnsolvedArmor &lb)
              { return abs((double)src.cols / 2 - ((la.left_light.top + la.right_light.top + la.left_light.bottom + la.right_light.bottom) / 4).x) < abs((double)src.cols / 2 - ((lb.left_light.top + lb.right_light.top + lb.left_light.bottom + lb.right_light.bottom) / 4).x); });
    for (auto armor : armors)
    {
        if (this->first_see)
        {
            this->first_see_time = this->time;
            this->first_see = 0;
        }
        int number = -1;
        convertNumber(armor.number, number);
        if (number == 7) // base
        {
            // continue;
        }
        Armor tar;
        pnp_solve(armor, ts, src, tar, number);
        tar_list.emplace_back(tar);
#ifdef DEBUGMODE
        int tickness{1};
        cv::line(src, tar.apex[0], tar.apex[1], cv::Scalar(0, 0, 255), tickness);
        cv::line(src, tar.apex[1], tar.apex[2], cv::Scalar(0, 0, 255), tickness);
        cv::line(src, tar.apex[2], tar.apex[3], cv::Scalar(0, 0, 255), tickness);
        cv::line(src, tar.apex[3], tar.apex[0], cv::Scalar(0, 0, 255), tickness);
        cv::circle(src, (tar.apex[0] + tar.apex[1] + tar.apex[2] + tar.apex[3]) / 4, 5, cv::Scalar(193, 182, 255), -1);
        cv::circle(src, tar.apex[0], 3, cv::Scalar(193, 182, 255), -1);
        cv::circle(src, tar.apex[1], 3, cv::Scalar(193, 182, 255), -1);
        cv::circle(src, tar.apex[2], 3, cv::Scalar(193, 182, 255), -1);
        cv::circle(src, tar.apex[3], 3, cv::Scalar(193, 182, 255), -1);
#endif // DEBUGMODE
    }

    armors_msg.armors = tar_list;

    //=====================输出装甲板坐标并且在DEBUGMODE ?展示识别到的装甲 ?======================//
    if (tar_list.size() != 0)
    {
        ts.message.x_a = tar_list[0].position(0) * 1000;
        ts.message.y_a = tar_list[0].position(1) * 1000;
        ts.message.z_a = tar_list[0].position(2) * 1000;
    }
    else
    {
        ts.message.x_a = 0;
        ts.message.y_a = 0;
        ts.message.z_a = 0;
    }
#ifdef DEBUGMODE
    showDist(tar_list, src);
    // cv::putText(src, std::to_string(shootKeeper), cv::Point(300, 300), 1, 2, cv::Scalar(225, 225, 0), 2);
    // cv::imshow("result", src);
#endif // DEBUGMODE
}
void AimAuto::pnp_solve(UnsolvedArmor &armor, Translator &ts, cv::Mat &src, Armor &tar, int number)
{
    //===============pnp解算===============//
    if (this->isBigArmor.empty())
    {
        this->isBigArmor.push_back(gp->isBigArmor[number]);
    }
    std::vector<cv::Point3f> objPoints;
    if (!gp->isBigArmor[number])
    { // 如果 ?小 ? 甲 ?
        if (gp->color == 1)
        {
            // printf("BLUE\n");
            objPoints = std::vector<cv::Point3f>{
                cv::Point3f(-67.50F, 28.50F, 0), // 2,3,4,1象限顺序
                cv::Point3f(-67.50F, -28.50F, 0),
                cv::Point3f(67.50F, -28.50F, 0),
                cv::Point3f(67.50F, 28.50F, 0),
            };
        }
        else
        {
            // printf("RED\n");
            objPoints = std::vector<cv::Point3f>{
                cv::Point3f(-67.50F, 28.5F, 0), // 2,3,4,1象限顺序
                cv::Point3f(-67.50F, -28.5F, 0),
                cv::Point3f(67.50F, -28.5F, 0),
                cv::Point3f(67.50F, 28.5F, 0),
            };
        }
    }
    else // 如果不是小 ? 甲 ?
        objPoints = std::vector<cv::Point3f>{
            cv::Point3f(-112.50F, 28.50F, 0), // 2,3,4,1象限顺序
            cv::Point3f(-112.50F, -28.50F, 0),
            cv::Point3f(112.50F, -28.50F, 0),
            cv::Point3f(112.50F, 28.50F, 0),
        };
    tVec.create(3, 1, CV_64F);
    rVec.create(3, 1, CV_64F);
    _K = (cv::Mat_<double>(3, 3) << (float)gp->fx, 0, (float)gp->cx, 0, (float)gp->fy, (float)gp->cy, 0, 0, 1);
    _dist = {(float)gp->k1, (float)gp->k2, (float)gp->p1, (float)gp->p2, (float)gp->k3};
    std::vector<cv::Point2f> tmp = {armor.left_light.top, armor.left_light.bottom, armor.right_light.bottom, armor.right_light.top};
    bool use_ippe_sq = 0;
    if (use_ippe_sq)
    {
        std::swap(objPoints[1], objPoints[3]);
        cv::solvePnP(objPoints, tmp, _K, _dist, rVec, tVec, false, cv::SOLVEPNP_IPPE_SQUARE);
    }
    else
    {
        cv::solvePnP(objPoints, tmp, _K, _dist, rVec, tVec, false, cv::SOLVEPNP_IPPE);
    }

    //=================坐标系转换================//
    tar.center = cv::Point3f(tVec.at<double>(0), tVec.at<double>(1), tVec.at<double>(2));
    cv::Mat rotation_matrix;
    double theta = rVec.at<double>(2);
    cv::Rodrigues(rVec, rotation_matrix);
    double yaw = std::atan2(rotation_matrix.at<double>(0, 2), rotation_matrix.at<double>(2, 2));
    if (yaw >= 0)
    {
        yaw = -(3.14 - yaw);
    }
    else
    {
        yaw = 3.14 + yaw;
    }
    //================数据 ? ?===================//

    tar.yaw = yaw;
    tar.angle = cv::Point3f(rVec.at<double>(0), rVec.at<double>(1), rVec.at<double>(2));
    tar.color = det->detect_color;
    tar.type = number;
    tar.apex[0] = armor.left_light.top;
    tar.apex[1] = armor.left_light.bottom;
    tar.apex[2] = armor.right_light.bottom;
    tar.apex[3] = armor.right_light.top;
    tar.distance_to_image_center = abs((double)src.cols / 2 - ((tar.apex[0] + tar.apex[1] + tar.apex[2] + tar.apex[3]) / 4).x);
#ifdef DEBUGMODE
    cv::putText(src, "PnpYaw:" + std::to_string(yaw), cv::Point(500, 200), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 255, 0), 2);
#endif
    if (number == 7)
    // base
    {
        tar.type = 10;
    }
    else if (number == 6)
    // sentry
    {
        tar.type = 7;
    }
    else if (number == 0)
    {
        tar.type = 11;
    }
    // ts.message.armor_flag = tar.type;
    Eigen::MatrixXd m_pitch(3, 3);
    Eigen::MatrixXd m_yaw(3, 3);
    this->raw_yaw = ts.message.yaw;
    ts.message.yaw = fmod(ts.message.yaw, 2 * M_PI);
    m_yaw << cos(ts.message.yaw), -sin(ts.message.yaw), 0, sin(ts.message.yaw), cos(ts.message.yaw), 0, 0, 0, 1;
    m_pitch << cos(ts.message.pitch), 0, sin(ts.message.pitch), 0, 1, 0, -sin(ts.message.pitch), 0, cos(ts.message.pitch);
    Eigen::Vector3d temp;
    temp = Eigen::Vector3d(tar.center.z / 1000 + VECTOR_X, -tar.center.x / 1000 + VECTOR_Y, -tar.center.y / 1000 + VECTOR_Z);
    tar.yaw -= ts.message.yaw;
    tar.yaw *= -1;
    tar.position = m_yaw * m_pitch * temp;
    // this->position_save = tar.center;
    // position is the world axis
    //=========================================//
}

void AimAuto::NewTracker(Translator &ts, cv::Mat &src)
{
//     if (restart_time == 0)
//     {
//         restart_time = this->time;
//         this->first_see = 1;
//     }
//     ts.message.armor_flag = 0;
//     if (armors_msg.armors.size() == 0)
//     {
//         last_ts = ts;
//         ts.message.status = 0;
//         if (this->shootKeeper > 0)
//         {
//             shootKeeper--;
//             ts.message.armor_flag = 11;
//         }

// #ifdef DEBUGMODE
//         if (ts.message.armor_flag >= 11)
//         {
//             cv::putText(src, "FIRE!", cv::Point(720 - 60, 540 - 80), cv::FONT_HERSHEY_PLAIN, 3, cv::Scalar(255, 255, 0), 3);
//         }
// #endif
//         return;
//     }
//     if (!this->updateTracker(ts, src))
//         return;
//     // auto p_ = findTarget(ts, time_add, src);
//     // storeMessage(p_, ts);

// #ifdef SENDCAMERA
//     // this->convertPoint(ts, p_, 1, src);
// #ifndef DEBUGMODE
//     printf("x:%.3lf|y:%.3lf|z:%.3lf\n", ts.message.x_a, ts.message.y_a, ts.message.z_a);
//     // printf("R:%.3lf|vyaw:%.3lf|Num:%d\n", this->r * (this->isClockwise > 0 ? 1.05 : 0.95), rawOmega, tracking_numb);
//     printf("yaw:%.4f|pitch:%.4f|flag:%d\n", ts.message.yaw, ts.message.pitch, ts.message.armor_flag);
//     printf("bcO:%.3lf|bcV:%.3lf\n", ts.message.vz_c, ts.message.vy_c);
// #endif
//     last_ts = ts;
//     if (this->shootKeeper > 0)
//     {
//         shootKeeper--;
//         ts.message.armor_flag = 11;
//     }
//     // printf("keeper:%d\n", shootKeeper);
// #ifdef DEBUGMODE
//     if (ts.message.armor_flag >= 11)
//     {
//         cv::putText(src, "FIRE!", cv::Point(720 - 60, 540 - 80), cv::FONT_HERSHEY_PLAIN, 3, cv::Scalar(255, 255, 0), 3);
//     }
// #endif
//     if (tracker_0->tracker_state == Tracker::LOST or tracker_0->tracker_state == Tracker::TEMP_LOST)
//     {
//         ts.message.armor_flag = 0;
//     }

//     this->closest2Armors.clear();
//     this->isBigArmor.clear();

// #endif // SENDCAMERA
}

// bool AimAuto::updateTracker(Translator &ts, cv::Mat &src)
// {
//     std::string t = "";
//     if (tracker_0->tracker_state == Tracker::LOST)
//     {
//         t = "LOST";
//     }
//     else if (tracker_0->tracker_state != Tracker::TEMP_LOST)
//     {
//         t = "TRAC";
//     }
//     cv::putText(src, t, cv::Point(50, 200), 1, 2, cv::Scalar(0, 255, 0));
//     if (!armors_msg.armors.empty())
//     {
//         if (tracker_0->tracker_state == Tracker::LOST)
//         {
//             dt = 0.01;
//             tracker_0->init(armors_msg);
//             tracker_1->init(armors_msg);
//         }
//         else
//         {
//             dt = this->time - this->last_time;
//             dt /= 1000;
//             int id = 0;
//             tracking_numb = tar_list[0].type;
//             tracker_0->update(armors_msg, src, tracking_numb);
//             tracker_1->update(armors_msg, src, tracking_numb);
//             last_time = this->time;
//         }
//         // std::cout << tracker_0->target_state << std::endl;
//         this->r = tracker_0->target_state(8);
//         return true;
//     }
//     else
//     {
//         if (tracker_0->tracker_state == Tracker::LOST)
//         {
//             return false;
//         }
//         else
//         {
//             time_add = this->time - this->last_time;
//             return true;
//         }
//     }
// }

void AimAuto::setTime(double t)
{
    this->receive_second = t;
}