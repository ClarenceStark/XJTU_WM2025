#ifndef _PREDICT_HPP
#define _PREDICT_HPP
#include "globalParam.hpp"
//#include <ceres/ceres.h>
#include <chrono>
#include <deque>
#include <fstream>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include "WMIdentify.hpp"
class WMIPredict
{
private:
    // 求解小符方程
    double f1(double P0, double fly_t0, double theta_0, double v0);
    double f2(double P0, double fly_t0, double theta_0, double v0);
    double f1P(double P, double fly_t, double theta_0, double v0);
    double f1t(double P, double fly_t, double theta_0, double v0);
    double f2P(double P, double fly_t, double theta_0, double v0);
    double f2t(double P, double fly_t, double theta_0, double v0);

    // 求解大符方程
    double F1(double P0, double fly_t0, double theta_0, double v0);
    double F2(double P0, double fly_t0, double theta_0, double v0);
    double F1P(double P, double fly_t, double theta_0, double v0);
    double F1t(double P, double fly_t, double theta_0, double v0);
    double F2P(double P, double fly_t, double theta_0, double v0);
    double F2t(double P, double fly_t, double theta_0, double v0);
    double ThetaToolForBig(double dt, double t0);
    // debug - 画图
    cv::Point2d CalPointGuess(double theta);
    cv::Mat debugImg;
    cv::Mat data_img;
    cv::Mat smoothData_img;
    cv::Point2d R_center;
    std::deque<double> y_data_s;
    std::deque<double> x_data_s;
    
    
    int direction;
    double Radius;    //能量机关半径，修正角度用
    double Fire_time;
    double First_fit; //是否为初次拟合1，0
    //====大符速度参数======//
    double A0;
    double w_big;
    double b;
    double fai;
    double now_time;
    double w;         // 小符角速度
   
public:
    WMIPredict();
    int StartPredict(Translator &translator, GlobalParam &gp,WMIdentify &WMI);

    void thetaAmend(double &theta);
    int BulletSpeedProcess(Translator &translator,GlobalParam &gp);
    void UpdateData(double direction, double Radius,cv::Point2d R_center,cv::Mat  debugImg,cv::Mat data_img,Translator translator);
    int Fit(std::deque<double> time_list, std::deque<double> angle_velocity_list, GlobalParam &gp, Translator &tr);
    void NewtonDspSmall(double theta_0, Translator &translator, GlobalParam &gp, double R_yaw);
    void NewtonDspBig(double theta_0, Translator &translator, GlobalParam &gp, double R_yaw);
    
    double Estim(double w, double &p1, double &p2, double &p3, std::deque<double> x_data, std::deque<double> y_data);
    void ConvexOptimization(std::deque<double> x_data, std::deque<double> y_data, GlobalParam &gp, Translator &tr);
    
    void ResultLog(Translator &translator, GlobalParam &gp, double R_yaw);
    void GiveDebugImg(cv::Mat debugImg);
    cv::Mat GetDebugImg();
    
};

#endif // _PREDICT_HPP