/*
 * @Author: zxh 1608278840@qq.com
 * @Date: 2023-11-08 03:11:49
 * @FilePath: /DX_aimbot/windmill/src/WMPredict.cpp
 * @Description:能量机关预测
 * Copyright (c) 2023 by ${git_name_email}, All Rights Reserved.
 */

#include "WMPredict.hpp"
#include "SerialPort.hpp"
#include "WMIdentify.hpp"
#include "globalParam.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc/types_c.h"
#include <algorithm>
//#include <ceres/loss_function.h>
#include <cmath>
#include <complex>
#include <deque>
#include <glog/logging.h>
#include <iostream>
#include <memory>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <ostream>
#include <unistd.h>
#include <vector>

//=====常量=====//
static double g = 9.8;       // 重力加速度
static double k = 0.05;      // 空气阻力系数
static double r = 0.7;       // 符的半径
static double s = 7.3;         // 车距离符的水平距离
static double Exp = 2.71823; // 自然常数e
static double h0 = 1.1747;   // 符的中心点高度减去车的高度
static double pi = 3.14159;  // 圆周率
static double delta_t = 0.1; //超前滞后
static double diff_w = 0.01;
static double w_low = 1.5;
static double w_up = 2.5;
static double yaw_fix = 0.07; // 相机yaw的修正常数
WMIPredict::WMIPredict()
{
    this->direction = 1;
    this->smoothData_img = cv::Mat::zeros(400, 800, CV_8UC3);
    //====大符速度参数======//
    this->A0 = 1;
    this->w_big = 2;
    this->b = 2.09 - A0;
    this->fai = 0;
    this->now_time = 0;
    this->Fire_time = 0;
    this->First_fit = 1;
}
/**
 * @description: 打符主流程
 * @param {Translator} &translator  串口消息
 * @param {GlobalParam} &gp  传入全局变量
 * @param {WMIdentify} &WMI  传入识别类
 * @return {*}
 */
int WMIPredict::StartPredict(Translator &translator, GlobalParam &gp, WMIdentify &WMI)
{

    if (WMI.getListStat() == 0)
    {
#ifdef THREADANALYSIS
        printf("WMI failed\n");
#endif
        LOG_IF(INFO, gp.switch_INFO) << "识别失败，不预测";
        return 0;
    }
    this->UpdateData(WMI.getDirection(), WMI.getRadius(), WMI.getR_center(), WMI.getImg0(), WMI.getData_img(), translator);
    // 如果击打大符
    if (translator.message.status % 5 == 3)
    {
        // 如果角速度数据数量够，进行拟合

        if (WMI.getAngleVelocityList().size() >= gp.list_size)
        {
            this->ConvexOptimization(WMI.getTimeList(), WMI.getAngleVelocityList(), gp, translator);
        }

        // 否则进入下一次循环，积累数据
        else
        {
            LOG_IF(INFO, gp.switch_INFO) << "数据不够，不拟合 " << WMI.getAngleVelocityList().size();
            return 0;
        }
        this->NewtonDspBig(WMI.getAngleList(), translator, gp, WMI.getYaw());
    }
    // 如果击打小符
    else
        this->NewtonDspSmall(WMI.getAngleList(), translator, gp, WMI.getYaw());

    translator.message.predict_time = WMI.GetFanChangeTime();
    this->ResultLog(translator, gp, WMI.getYaw());
#ifdef DEBUGHIT
    cv::imshow("Energy_debug", this->debugImg);
    cv::waitKey(1);
#endif
    return 1;
}
/**
 * @description: 根据识别结果对预测的数据更新
 * @param {double} direction 能量机关旋转方向
 * @param {double} Radius   能量机关半径
 * @param {Point2d} R_center    能量机关中心点
 * @param {Mat} debugImg        当前识别的图片
 * @param {Mat} data_img        角速度队列绘制结果
 * @param {Translator} translator
 * @return {*}
 */
void WMIPredict::UpdateData(double direction, double Radius, cv::Point2d R_center, cv::Mat debugImg, cv::Mat data_img, Translator translator)
{
    this->w = abs(direction) > 20 ? 1.047197551 : 0; // 小符角速度
    // this->w = 0;
    this->direction = direction > 0 ? 1 : -1;
    this->R_center = R_center;
    this->Radius = Radius;
    this->now_time = (double)translator.message.predict_time / 1000;
    this->debugImg = debugImg;
    this->data_img = data_img;
}

/**
 * @description: 迭代求解子弹飞行时间与云台下一时刻开火的位姿
 * @param {double} theta_0  当前能量机关角度
 * @param {Translator} &translator
 * @param {GlobalParam} &gp
 * @param {double} R_yaw   当前云台相对于R标（能量机关中心）的yaw角
 * @return {*}
 */
void WMIPredict::NewtonDspSmall(double theta_0, Translator &translator, GlobalParam &gp, double R_yaw)
{
    double P0 = 12 * pi / 180;
    double fly_t0 = 0.3;
    w = this->direction > 0 ? abs(w) : -abs(w);
    int n = 0;                                                                          // 迭代次数
    translator.message.predict_time = translator.message.predict_time + 1000 * delta_t; // 开火时间
    theta_0 = theta_0 + w * delta_t;                                                    // 开火时的待击打点角度
    theta_0 += theta_0 < 0 ? 2 * pi : 0;
    theta_0 -= theta_0 > 2 * pi ? 2 * pi : 0; // theta0 范围锁定（0，2pi）
                                              // std::cout<<"theta: "<<180*theta_0/pi<<std::endl;
    double v0 = translator.message.bullet_v;  // 弹速
    cv::Mat P_t = cv::Mat::zeros(2, 1, CV_64F);
    cv::Mat temp = (cv::Mat_<double>(2, 2) << this->f1P(P0, fly_t0, theta_0, v0), this->f1t(P0, fly_t0, theta_0, v0), this->f2P(P0, fly_t0, theta_0, v0), this->f2t(P0, fly_t0, theta_0, v0));
    cv::Mat temp_inv = cv::Mat::zeros(2, 2, CV_64F);
    cv::Mat b = (cv::Mat_<double>(2, 1) << this->f1(P0, fly_t0, theta_0, v0), this->f2(P0, fly_t0, theta_0, v0));
    double P1 = 0;
    double fly_t1 = 0;
    do
    {
        n++;
        P1 = P0;
        fly_t1 = fly_t0;
        // std::cout<<"P0:"<<P0；
        // std::cout<<" t0:"<<t0<<std::endl;
        //======这里对雅可比矩阵的更新要尽可能的少，不然解变化太快容易求出无意义解（t<0)======//
        temp.at<double>(0, 0) = this->f1P(P0, fly_t0, theta_0, v0);
        // temp.at<double>(0, 1) = f1t(P0, t0, theta_0,v0);
        // temp.at<double>(1, 0) = f2P(P0, t0, theta_0,v0);
        temp.at<double>(1, 1) = this->f2t(P0, fly_t0, theta_0, v0);
        // std::cout<<"temp: "<<temp<<std::endl;
        cv::invert(temp, temp_inv);
        // std::cout<<"temp_inv: "<<temp_inv<<std::endl;
        b.at<double>(0, 0) = this->f1(P0, fly_t0, theta_0, v0);
        b.at<double>(1, 0) = this->f2(P0, fly_t0, theta_0, v0);
        P_t = P_t - temp_inv * b;
        P0 = P_t.at<double>(0, 0);
        fly_t0 = P_t.at<double>(1, 0);
        if (n > 50)
            break;
    } while (abs(fly_t0 - fly_t1) > 1e-5 || abs(P0 - P1) > 1e-5); // 当前解与上次迭代解差距很小时

    double yaw = atan(r * cos(theta_0 + w * fly_t0) / s);
    // cv::putText(this->debugImg, "yaw:" + std::to_string(180*yaw/pi), cv::Point(30, 30), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 100, 0));
#ifdef DEBUGHIT

    double theta_guess = theta_0 + w * fly_t0;
    cv::circle(this->debugImg, CalPointGuess(theta_0), 5, cv::Scalar(0, 255, 255), -1);
    cv::circle(this->debugImg, CalPointGuess(theta_guess), 5, cv::Scalar(255, 0, 255), -1);

#endif
    translator.message.x_a = translator.message.yaw;
    translator.message.pitch = 180 * P0 / pi;
    translator.message.yaw = 180 * (yaw + R_yaw) / pi;
    // std::cout<< translator.message.yaw<<std::endl;
    // 相机中心的偏置修正
    translator.message.yaw += yaw_fix;
    // std::cout << " 开火时待打击点角度： " << 180 / pi * theta_0;
    LOG_IF(INFO, gp.switch_INFO) << "小符角速度" << w;
    LOG_IF(INFO, gp.switch_INFO) << " 开火时待打击点角度： " << 180 / pi * theta_0;
    LOG_IF(INFO, gp.switch_INFO) << " 对应的子弹飞行时间 " << fly_t0;
    LOG_IF(INFO, gp.switch_INFO) << " " << delta_t << "s后要调整的相对R的yaw: " << 180 / pi * yaw;
    LOG_IF(INFO, gp.switch_INFO) << " " << delta_t << "s后要调整的pitch: " << translator.message.pitch;
}
void WMIPredict::NewtonDspBig(double theta_0, Translator &translator, GlobalParam &gp, double R_yaw)
{
    double P0 = 12 * pi / 180;
    double fly_t0 = 0.3;
    int n = 0;                                                    // 迭代次数
    theta_0 = theta_0 + ThetaToolForBig(delta_t, this->now_time); // 开火时的待击打点角度
                                                                  // std::cout<<"time:"<<now_time<<std::endl;
    this->Fire_time = this->now_time + delta_t;
    theta_0 += theta_0 < 0 ? 2 * pi : 0;
    theta_0 -= theta_0 > 2 * pi ? 2 * pi : 0;
    std::cout << "theta:" << theta_0 << std::endl;
    std::cout << "w_big:" << w_big << std::endl;
    double v0 = translator.message.bullet_v; // 弹速
    cv::Mat P_t = cv::Mat::zeros(2, 1, CV_64F);
    cv::Mat temp = (cv::Mat_<double>(2, 2) << this->F1P(P0, fly_t0, theta_0, v0), this->F1t(P0, fly_t0, theta_0, v0), this->F2P(P0, fly_t0, theta_0, v0), this->F2t(P0, fly_t0, theta_0, v0));
    cv::Mat temp_inv = cv::Mat::zeros(2, 2, CV_64F);
    cv::Mat b = (cv::Mat_<double>(2, 1) << this->F1(P0, fly_t0, theta_0, v0), this->F2(P0, fly_t0, theta_0, v0));
    double P1 = 0;
    double fly_t1 = 0;
    do
    {
        n++;
        P1 = P0;
        fly_t1 = fly_t0;
        //======这里对雅可比矩阵的更新要尽可能的少，不然解变化太快容易求出无意义解（t<0)======//
        temp.at<double>(0, 0) = this->F1P(P0, fly_t0, theta_0, v0);
        // temp.at<double>(0, 1) = f1t(P0, t0, theta_0,v0);
        // temp.at<double>(1, 0) = f2P(P0, t0, theta_0,v0);
        temp.at<double>(1, 1) = this->F2t(P0, fly_t0, theta_0, v0);
        cv::invert(temp, temp_inv);
        b.at<double>(0, 0) = this->F1(P0, fly_t0, theta_0, v0);
        b.at<double>(1, 0) = this->F2(P0, fly_t0, theta_0, v0);
        P_t = P_t - temp_inv * b;
        P0 = P_t.at<double>(0, 0);
        fly_t0 = P_t.at<double>(1, 0);
        if (n > 50)
            break;
    } while (abs(fly_t0 - fly_t1) > 1e-5 || abs(P0 - P1) > 1e-5); // 当前解与上次迭代解差距很小时
    double yaw = atan(r * cos(theta_0 + ThetaToolForBig(fly_t0, this->Fire_time)) / s);
#ifdef DEBUGHIT
    double theta_guess = ThetaToolForBig(fly_t0, Fire_time) + theta_0;
    // double theta_fit_debug = ThetaToolForBig(2 * pi / w_big - delta_t, Fire_time) + theta_0;
    // cv::circle(this->debugImg, CalPointGuess(theta_fit_debug), 5, cv::Scalar(125, 125, 255), -1);
    cv::circle(this->debugImg, CalPointGuess(theta_0), 5, cv::Scalar(0, 255, 255), -1);
    cv::circle(this->debugImg, CalPointGuess(theta_guess), 5, cv::Scalar(255, 0, 255), -1);
    // cv::putText(this->debugImg, "points' distance:" + std::to_string(sqrt(CalDistSquare(CalPointGuess(theta_fit_debug), CalPointGuess(theta_0 - ThetaToolForBig(delta_t, this->now_time))))), cv::Point(30, 500), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 0));

#endif

    translator.message.x_a = translator.message.yaw;
    translator.message.pitch = P0 * 180 / pi;
    // std::cout<<translator.message.pitch<<std::endl;
    std::cout << fly_t0 << std::endl;
    translator.message.yaw = (yaw + R_yaw) * 180 / pi;
    translator.message.yaw += yaw_fix;

    // std::cout<<"  speed_now: "<<A0*sin(w_big*(double)translator.message.predict_time/1000+fai)+this->b;
    translator.message.predict_time = translator.message.predict_time + 1000 * delta_t; // 发给电控的开火时间

    // std::cout<< " 对应的子弹飞行时间 " << fly_t0;
    // std::cout<< " " << delta_t << "s后要调整的相对R的yaw: " << 180 / pi * yaw;
    LOG_IF(INFO, gp.switch_INFO) << " 对应的子弹飞行时间 " << fly_t0;
    LOG_IF(INFO, gp.switch_INFO) << " " << delta_t << "s后要调整的相对R的yaw: " << 180 / pi * yaw;
    LOG_IF(INFO, gp.switch_INFO) << " " << delta_t << "s后要调整的pitch: " << translator.message.pitch;
    LOG_IF(INFO, gp.switch_INFO) << " 开火时待打击点角度： " << 180 / pi * theta_0;
    LOG_IF(INFO, gp.switch_INFO) << " 对应的子弹飞行时间 " << fly_t0;
    LOG_IF(INFO, gp.switch_INFO) << " " << delta_t << "s后要调整的相对R的yaw: " << 180 / pi * yaw;
    LOG_IF(INFO, gp.switch_INFO) << " " << delta_t << "s后要调整的pitch: " << translator.message.pitch;
}
/**
 * @description: 输入w后，进行P参数的最优估计
 * @param {double} w
 * @param {double} &p1
 * @param {double} &p2
 * @param {double} &p3
 * @param {deque<double>} x_data  时间队列
 * @param {deque<double>} y_data  角速度队列
 * @return {double} 返回残差
 */
double WMIPredict::Estim(double w, double &p1, double &p2, double &p3, std::deque<double> x_data, std::deque<double> y_data)
{
    std::vector<double> x1;
    std::vector<double> x2;
    std::vector<double> temp1;
    std::vector<double> temp2;
    std::vector<double> temp3;
    for (auto x : x_data)
    {
        x2.push_back(cos(w * x));
        x1.push_back(sin(w * x));
    }

    // 对两个向量进行操作
    std::transform(x1.begin(), x1.end(), x2.begin(),
                   std::back_inserter(temp1),
                   [](double a, double b)
                   { return a * b; });
    std::transform(x1.begin(), x1.end(), y_data.begin(),
                   std::back_inserter(temp2),
                   [](double a, double b)
                   { return a * b; });
    std::transform(x2.begin(), x2.end(), y_data.begin(),
                   std::back_inserter(temp3),
                   [](double a, double b)
                   { return a * b; });

    // 最小二乘法的求和
    double sum_x1x2 = std::accumulate(temp1.begin(), temp1.end(), 0.0);
    double sum_x1y = std::accumulate(temp2.begin(), temp2.end(), 0.0);
    double sum_x2y = std::accumulate(temp3.begin(), temp3.end(), 0.0);
    double sum_y = std::accumulate(y_data.begin(), y_data.end(), 0.0);
    double sum_x1 = std::accumulate(x1.begin(), x1.end(), 0.0);
    double sum_x2 = std::accumulate(x2.begin(), x2.end(), 0.0);
    double sum_x1x1 = std::accumulate(x1.begin(), x1.end(), 0.0, [](double a, double b)
                                      { return a + b * b; });
    double sum_x2x2 = std::accumulate(x2.begin(), x2.end(), 0.0, [](double a, double b)
                                      { return a + b * b; });

    // 定义矩阵
    cv::Mat_<double> A(3, 3);
    cv::Mat_<double> b(3, 1);
    A << sum_x1x1, sum_x1x2, sum_x1, sum_x1x2, sum_x2x2, sum_x2, sum_x1, sum_x2, y_data.size();
    b << sum_x1y, sum_x2y, sum_y;
    // 求解 Ax=b
    cv::Mat_<double> x = A.inv() * b;
    p1 = x(0);
    p2 = x(1);
    p3 = x(2);
    // 求出残差和
    double err_sum = 0;
    for (int i = 0; i < x1.size(); i++)
    {
        err_sum += abs(x1[i] * p1 + x2[i] * p2 + p3 - y_data[i]);
    }

    return err_sum;
}
/**
 * @description: 拟合函数，遍历w选取残差最小参数组
 * @param {deque<double>} x_data
 * @param {deque<double>} y_data
 * @param {GlobalParam} &gp
 * @param {Translator} &tr
 * @return {*}
 */
void WMIPredict::ConvexOptimization(std::deque<double> x_data, std::deque<double> y_data, GlobalParam &gp, Translator &tr)
{
    // 自己准备数据点测试拟合效果
#ifdef DEBUGHIT
    // x_data.clear();
    // y_data.clear();
    // std::ifstream inputFile("/home/xjturm/DX_aimbot/points0.txt");
    // double var1, var2;
    // std::string line;
    // while (std::getline(inputFile, line))
    // {
    //     std::istringstream iss(line);

    //     if (iss >> var1 >> var2)
    //     {
    //         static double count = 0;
    //         count++;
    //         if (count > 300)
    //             break;
    //         x_data.push_back(var1);
    //         y_data.push_back(std::abs(var2));
    //     }
    // }
    // inputFile.close();
#endif // DEBUGHIT
    for (int i = 0; i < y_data.size(); i++)
    {
        y_data[i] = abs(y_data[i]);
    }
    double w0 = 0;
    double p1 = 0;
    double p2 = 0;
    double p3 = 0;
    double err_min = 0;
    std::vector<double> err_list;

    std::once_flag once;
    double p1_temp;
    double p2_temp;
    double p3_temp;
    double err_temp;

    // 遍历所有 w 找出最小的残差对应的 w
    for (double w_temp = w_low; w_temp < w_up; w_temp += diff_w)
    {
        err_temp = Estim(w_temp, p1_temp, p2_temp, p3_temp, x_data, y_data);
        // std::cout << err_temp << std::endl;
        if (err_list.size() != 0)
        {

            std::call_once(once, [&]()
                           { err_min = err_list[0];  
                           w0 = w_temp;
                            p1 = p1_temp;
                            p2 = p2_temp;
                            p3 = p3_temp; });
            if (err_temp < err_min)
            {
                err_min = err_temp;
                w0 = w_temp;
                p1 = p1_temp;
                p2 = p2_temp;
                p3 = p3_temp;
            }
        }
        err_list.push_back(err_temp);
    }

    this->w_big = w0;
    this->fai = atan2(p2, p1);
    this->A0 = p1 / cos(fai);
    this->b = p3;

#ifdef DEBUGHIT
    cv::Mat word_show = cv::Mat::zeros(700, 800, CV_8UC3);
    cv::putText(word_show, "time:" + std::to_string(tr.message.predict_time / 1000), cv::Point(30, 390), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 0));
    // cv::putText(word_show, "final_cost:" + std::to_string(sun), cv::Point(30, 330), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 0));
    cv::putText(word_show, "A0:" + std::to_string(this->A0), cv::Point(30, 420), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 0));
    cv::putText(word_show, "w0:" + std::to_string(this->w_big), cv::Point(30, 450), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 0));
    cv::putText(word_show, "fai0:" + std::to_string(this->fai), cv::Point(30, 480), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 0));
    cv::putText(word_show, "b:" + std::to_string(this->b), cv::Point(30, 510), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 0));
    cv::putText(word_show, "p1:" + std::to_string(p1), cv::Point(30, 540), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 0));
    cv::putText(word_show, "p2:" + std::to_string(p2), cv::Point(30, 570), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 0));
    cv::putText(word_show, "p3:" + std::to_string(p3), cv::Point(30, 600), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 0));
    cv::putText(word_show, "err_min:" + std::to_string(err_min), cv::Point(30, 630), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 0));
    cv::putText(word_show, "err_judge:" + std::to_string(err_min / y_data.size()), cv::Point(30, 660), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 0));
    cv::imshow("word_show", word_show);

    cv::Point2f first_point(x_data[0], abs(y_data[0]));
    cv::Mat data_img = cv::Mat::zeros(1080, 1440, CV_8UC3);
    for (int i = 0; i < x_data.size(); i++)
    {

        cv::Point2f now_point(x_data[i], abs(y_data[i]));
        cv::Point2f now_point_fit(x_data[i], abs(y_data[i]));

        now_point.x -= first_point.x;
        now_point.x *= 20;
        now_point.y = now_point.y * 100;

        now_point_fit.x -= first_point.x;
        now_point_fit.x *= 20;
        now_point_fit.y = (A0 * sin(w0 * x_data[i] + fai) + b) * 100;

        cv::circle(data_img, now_point, 1, cv::Scalar(0, 255, 255));
        // cv::circle(this->smoothData_img, now_point_fit, 1, cv::Scalar(0, 0, 255));
        cv::circle(data_img, now_point_fit, 1, cv::Scalar(0, 0, 255));
    }
    cv::imshow("initial", data_img);
#endif // DEBUGHIT
    LOG_IF(INFO, gp.switch_INFO) << "estimated A:" << this->A0;
    LOG_IF(INFO, gp.switch_INFO) << "estimated w:" << this->w_big;
    LOG_IF(INFO, gp.switch_INFO) << "estimated sketchy:fai:" << this->fai << std::endl;
}
/**
 * @description: 预测结果日志输出
 * @param {Translator} &translator
 * @param {GlobalParam} &gp
 * @param {double} R_yaw  当前云台相对于R标（能量机关中心）的yaw角
 * @return {*}
 */
void WMIPredict::ResultLog(Translator &translator, GlobalParam &gp, double R_yaw)
{
#ifdef DEBUGHIT
    cv::putText(this->debugImg, "bullet_v:" + std::to_string(translator.message.bullet_v), cv::Point(30, 60), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 100, 0));

    cv::putText(this->debugImg, "delta_t:" + std::to_string(delta_t), cv::Point(30, 90), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 100, 0));
    cv::putText(this->debugImg, "pitch:" + std::to_string(translator.message.pitch), cv::Point(30, 120), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 100, 0));
    cv::putText(this->debugImg, "delta_yaw:" + std::to_string(translator.message.yaw), cv::Point(30, 150), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 100, 0));
    cv::putText(this->debugImg, "R_yaw:" + std::to_string(R_yaw * 180 / pi), cv::Point(30, 180), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 100, 0));
    cv::putText(this->debugImg, "w:" + std::to_string(w), cv::Point(30, 210), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 100, 0));
#endif
    LOG_IF(INFO, gp.switch_INFO) << "当前状态: " << +translator.message.status;
    LOG_IF(INFO, gp.switch_INFO) << "拍照时(" << translator.message.predict_time << ")云台相对于中心点R的yaw: " << 180 / pi * R_yaw;
    LOG_IF(INFO, gp.switch_INFO) << "预测时间戳: " << translator.message.predict_time;
}

double WMIPredict::ThetaToolForBig(double dt, double t0) // 计算t0->t0+dt的大符角度
{
    return this->direction * (this->b * dt + this->A0 / this->w_big * (cos(this->w_big * t0 + this->fai) - cos(this->w_big * (t0 + dt) + this->fai)));
}
cv::Point2d WMIPredict::CalPointGuess(double theta)
{
    // 注意opencv坐标系
    cv::Point2d point_guess(R_center.x + Radius * cos(theta), R_center.y - Radius * sin((theta)));
    return point_guess;
}

double WMIPredict::f1(double P0, double fly_t0, double theta_0, double v0)
{
    return sqrt(pow(r * cos(theta_0 + w * fly_t0), 2) + s * s) - v0 * cos(P0) / k + v0 / k * cos(P0) * pow(Exp, -k * fly_t0);
}

double WMIPredict::f2(double P0, double fly_t0, double theta_0, double v0)
{
    return h0 + r * sin(theta_0 + w * fly_t0) - (k * v0 * sin(P0) + g - (k * v0 * sin(P0) + g) * pow(Exp, -k * fly_t0) - g * k * fly_t0) / (k * k);
}

double WMIPredict::f1P(double P, double fly_t, double theta_0, double v0) // f1关于p的导数
{
    return v0 * sin(P) / k * (1 - pow(Exp, -k * fly_t));
}
double WMIPredict::f1t(double P, double fly_t, double theta_0, double v0) // f1关于t的导数
{
    return (-r * r * w * cos(theta_0 + w * fly_t) * sin(theta_0 + w * fly_t)) / sqrt(pow(r * cos(theta_0 + w * fly_t), 2) + s * s) - v0 * cos(P) * pow(Exp, -k * fly_t);
}
double WMIPredict::f2P(double P, double fly_t, double theta_0, double v0) // f2关于p的导数
{
    return v0 * cos(P) / k * (pow(Exp, -k * fly_t) - 1);
}
double WMIPredict::f2t(double P, double fly_t, double theta_0, double v0) // f2关于t的导数
{
    return w * r * cos(theta_0 + w * fly_t) - (k * v0 * sin(P) + g) * pow(Exp, -k * fly_t) / k + g / k;
}

double WMIPredict::F1(double P0, double fly_t0, double theta_0, double v0)
{
    return sqrt(pow(r * cos(theta_0 + ThetaToolForBig(fly_t0, this->Fire_time)), 2) + s * s) - v0 * cos(P0) / k + v0 / k * cos(P0) * pow(Exp, -k * fly_t0);
}
double WMIPredict::F2(double P0, double fly_t0, double theta_0, double v0)
{
    return h0 + r * sin(theta_0 + ThetaToolForBig(fly_t0, this->Fire_time)) - (k * v0 * sin(P0) + g - (k * v0 * sin(P0) + g) * pow(Exp, -k * fly_t0) - g * k * fly_t0) / (k * k);
}
double WMIPredict::F1P(double P, double fly_t, double theta_0, double v0) // F1关于p的导数
{
    return v0 * sin(P) / k * (1 - pow(Exp, -k * fly_t));
}
double WMIPredict::F1t(double P, double fly_t, double theta_0, double v0) // f1关于t的导数
{
    return (-r * r * w * cos(theta_0 + ThetaToolForBig(fly_t, this->Fire_time)) * sin(theta_0 + ThetaToolForBig(fly_t, this->Fire_time))) / sqrt(pow(r * cos(theta_0 + ThetaToolForBig(fly_t, this->Fire_time)), 2) + s * s) - v0 * cos(P) * pow(Exp, -k * fly_t);
}
double WMIPredict::F2P(double P, double fly_t, double theta_0, double v0) // f2关于p的导数
{
    return v0 * cos(P) / k * (pow(Exp, -k * fly_t) - 1);
}
double WMIPredict::F2t(double P, double fly_t, double theta_0, double v0) // f2关于t的导数
{
    return w * r * cos(theta_0 + ThetaToolForBig(fly_t, this->Fire_time)) - (k * v0 * sin(P) + g) * pow(Exp, -k * fly_t) / k + g / k;
}

cv::Mat WMIPredict::GetDebugImg()
{
    return this->debugImg;
}
void WMIPredict::GiveDebugImg(cv::Mat debugImg)
{
    this->debugImg = debugImg;
}
/**
 * @description: 弹速单位转化为m/s,同时判断弹速能否激活能量机关
 * @param {Translator} &translator
 * @param {GlobalParam} &gp
 * @return {*}
 */
int WMIPredict::BulletSpeedProcess(Translator &translator, GlobalParam &gp)
{
    translator.message.bullet_v /= 10;
    // 弹速过小时无法命中
    if (translator.message.bullet_v < gp.min_bullet_v)
    {
        std::cout << "弹速小" << std::endl;
        LOG_IF(INFO, gp.switch_INFO) << "弹速小，为：" << translator.message.bullet_v;
        return 0;
    }
    return 1;
}
