/**
 * @file WMIdentify.cpp
 * @author axi404 (3406402603@qq.com)
 * @brief 打符识别类实现，传统方法
 * @version 0.1
 * @date 2022-12-30
 *
 * @copyright Copyright (c) 2022
 */
#include "WMIdentify.hpp"
#include "globalParam.hpp"
#include "opencv2/core/hal/interface.h"
#include "opencv2/core/types.hpp"
#include <algorithm>
#include <chrono>
#include <deque>
#include <glog/logging.h>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <iostream>
using namespace std;
#define Pi 3.1415926
WMIdentify::WMIdentify(GlobalParam &gp)
{
    // std::cout << "喵～" << std::endl;
    this->gp = &gp;
    this->t_start = std::chrono::system_clock::now().time_since_epoch().count() / 1e9;
    // 从gp中读取一些数据
    this->switch_INFO = this->gp->switch_INFO;
    this->switch_ERROR = this->gp->switch_ERROR;
    this->get_armor_mode = this->gp->get_armor_mode;
    // 将R与Wing的状态设置为没有读取到内容
    this->R_stat = 0;
    this->Wing_stat = 0;
    this->Winghat_stat = 0;
    this->R_emitate.x = 0;
    this->R_emitate.y = 0;
    this->data_img = cv::Mat::zeros(400, 800, CV_8UC3);
    this->objectPoints = {
            cv::Point3f(0, 0, 0),
            cv::Point3f(0, 0.3733, 1.120),
            cv::Point3f(0, 0.3422, 1.736),
            cv::Point3f(0, -0.3422, 1.736),
            cv::Point3f(0, -0.3733, 1.120)
        };
    this->_K_ = (cv::Mat_<double>(3, 3) << (float)2400, 0, (float)720,
                0, (float)2400, (float)540,
                0, 0, 1 );
    // 输出日志，初始化成功
    LOG_IF(INFO, this->switch_INFO) << "WMIdentify Successful";
}

WMIdentify::~WMIdentify()
{
    // WMIdentify之中的内容都会自动析构
    // 输出日志，析构成功
    LOG_IF(INFO, this->switch_INFO) << "~WMIdentify Successful";
}

void WMIdentify::clear()
{
    // 清空队列中的内容
    this->wing_center_list.clear();
    this->wing_idx.clear();
    this->R_center_list.clear();
    this->R_idx.clear();
    this->time_list.clear();
    this->angle_list.clear();
    this->angle_velocity_list.clear();
    this->angle_velocity_list.emplace_back(0); // 先填充一个0，方便之后UpdateList中的数据对齐
    // 输出日志，清空成功
    LOG_IF(INFO, this->switch_INFO) << "clear Successful";
}

void WMIdentify::startWMINet(cv::Mat &input_img, Translator &ts)
{
    // 输入图片
    this->receive_pic(input_img);
   std::vector<WMObject> objects;
    detector.detect(this->img, objects);
      
    this->list_stat = 0;
    // 对置信度排序
    std::sort(objects.begin(), objects.end(), [this](WMObject &a, WMObject &b)
              { return a.prob < b.prob; });
    for (auto object : objects)
    {

        std::cout<<"cls: "<<object.cls<<std::endl;
        std::cout<<object.color<<std::endl;
        if (object.cls == 0 && ((object.color == 0 && gp->color == RED) || (object.color == 0 && gp->color == BLUE)) && (object.cls >= 0 && object.cls <= 5))
        {
          
#if defined(DEBUGMODE) or defined(DEBUGHIT)
            cv::line(this->img, object.apex[0], object.apex[1], cv::Scalar(193, 182, 255), 3);
            cv::line(this->img, object.apex[1], object.apex[2], cv::Scalar(193, 182, 255), 3);
            cv::line(this->img, object.apex[2], object.apex[3], cv::Scalar(193, 182, 255), 3);
            cv::line(this->img, object.apex[3], object.apex[0], cv::Scalar(193, 182, 255), 3);
            cv::circle(this->img, (object.apex[3] + object.apex[2] + object.apex[1]) / 3, 5, cv::Scalar(0, 0, 255), -1);
#endif // DEBUGMODE

        // cv::Mat rVec, tVec;
        // cv::solvePnP(this->objectPoints, imagePoints,this-> _K_, cv::Mat(), rVec, tVec);
        // cv::Mat rotation_matrix;
        // cv::Rodrigues(rVec, rotationMatrix);
    


            this->list_stat = 1;
        }
    }
   
    this->preprocess();
    //  std::chrono::milliseconds end_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    //         std::chrono::system_clock::now().time_since_epoch());
    //     float total_time = (end_ms - start_ms).count() / 1000.0;
    //     std::cout << "idfpss: " << 1 / total_time << std::endl;
    // 获取轮廓
    // cout<< "------------------------------------------------------------"<<endl;
    this->getContours();

    // 通过图像特征寻找R
    this->getVaildR();
    // 通过图像位置确定R
    this->selectR();

    if (this->R_center_list.size() >= 2)
    {
        this->R_center_list.pop_front();
    }
    // 如果寻找R中心点确实找到东西了，列表不为空，则把找到的R中心点坐标压进队列中，否则输出日志，数据不足
    if (this->R_idx.size() > 0 && this->R_contours.size() > 0 && list_stat == 1)
    {
        this->R_center_list.emplace_back(cv::minAreaRect(this->R_contours[this->R_idx[0]]).center);
    }
    else
    {
        LOG_IF(ERROR, this->switch_ERROR) << "failed to emplace_back R_center_list, lack data";
        if (list_stat == 1)
        {
            // 此时armor已经输入了数据，但是R无法输入数据，所以要把armor的数据吐出来，保证数据同步
            this->wing_center_list.pop_back();
        }
        list_stat = 0;
    }
    this->updateList((double)ts.message.predict_time / 1000);
          
}

void WMIdentify::receive_pic(cv::Mat &input_img)
{
    this->img = input_img;

    LOG_IF(INFO, this->switch_INFO) << "receive_pic Successful";
}

void WMIdentify::preprocess()
{
    
#ifdef DEBUGHIT
    this->img_0 = this->img.clone();

#endif // DEBUGHIT
    // 将蒙板中需要取到的区域（在globalParam中调整）内的像素点变为白色
   

#ifdef DEBUGMODE
// NOTE: 将来优化代码可以减少克隆次数
    cv::Mat mask(img.rows, img.cols, CV_8UC1, cv::Scalar(0)); //!< 创建的单通道图像，用于蒙板,初始全黑
    cv::Mat dst1;
    // 设置感兴趣区域（ROI）为白色
    mask(cv::Rect(this->img.cols * this->gp->mask_TL_x, this->img.rows * this->gp->mask_TL_y, this->img.cols * this->gp->mask_width, this->img.rows * this->gp->mask_height)) = 255;
    // 实现蒙板，将两边与能量机关无关的部分去掉
    this->img.copyTo(dst1, mask); //!< 用于储存蒙板之后的图片
    // cv::imshow("after mask", dst1);
    if (this->gp->switch_gaussian_blur == ON)
    {
        cv::GaussianBlur(dst1, dst1, cv::Size(5, 5), 0.0, 0.0);
    }
    // hsv二值化，此部分的参数对于后续的操作尤为重要

    cv::cvtColor(dst1, dst1, cv::COLOR_BGR2HSV);
    cv::inRange(dst1, cv::Scalar(this->gp->hmin, this->gp->smin, this->gp->vmin), cv::Scalar(this->gp->hmax, this->gp->smax, this->gp->vmax), this->binary);

#else
    if (this->gp->switch_gaussian_blur == ON)
    {
        cv::GaussianBlur(this->img, this->img, cv::Size(5, 5), 0.0, 0.0);
    }
    cv::cvtColor(this->img, this->img, cv::COLOR_BGR2HSV);
    cv::inRange(this->img, cv::Scalar(this->gp->hmin, this->gp->smin, this->gp->vmin), cv::Scalar(this->gp->hmax, this->gp->smax, this->gp->vmax), this->binary);

#endif
#ifdef DEBUGMODE
    // 展示预处理之后的binary
    cv::imshow("binary after preprocess", this->binary);
#endif // DEBUGMODE

    // 日志输出预处理成功
    LOG_IF(INFO, this->switch_INFO == ON) << "preprocess successful";
}

void WMIdentify::getContours()
{
    // 清空轮廓信息
    this->R_contours.clear();
    this->wing_contours.clear();
    cv::Mat binary_clone = this->binary;
    // 进行图形学操作，最后新添加的膨胀后腐蚀可以让灯带连在一起
    cv::dilate(binary_clone, binary_clone, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(this->gp->dialte1, this->gp->dialte1)));
    // 检测最外围轮廓，储存找到的轮廓至R_contours，用于R的识别
    cv::findContours(binary_clone, this->R_contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

#ifdef DEBUGMODE
    cv::imshow("R", binary_clone);
#endif // DEBUGMODE
#ifndef USEWMNET
    // 进行获取装甲板需要的图形学操作
    cv::dilate(binary_clone, binary_clone, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(this->gp->dialte2, this->gp->dialte2)));
    cv::dilate(binary_clone, binary_clone, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(this->gp->dialte3, this->gp->dialte3)));
    // 提取binary中所有轮廓，且hierarchy是以“树状结构”来进行组织，储存找到的轮廓至wing_contours，用于装甲板的识别
    cv::findContours(binary_clone, this->wing_contours, this->hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
#endif
#ifdef DEBUGMODE
    cv::imshow("armor", binary_clone);
#endif // DEBUGMODE
    // 日志输出获得轮廓成功成功
    LOG_IF(INFO, this->switch_INFO == ON) << "getContours successful";
}


int WMIdentify::getVaildR()
{
    this->R_idx.clear();
    // 遍历全部的轮廓，通过轮廓特征判断是否是R
    for (int i = 0; i < this->R_contours.size(); i++)
    {
        if (IsVaildR(this->R_contours[i], *gp))
        {
            this->R_idx.emplace_back(i);
        }
    }
        
    if (this->R_idx.size() == 0)
    {
        LOG_IF(INFO, this->switch_INFO) << "getVaildR Get No R";
        this->R_stat = 0;
    }
    else if (this->R_idx.size() == 1)
    {
        LOG_IF(INFO, this->switch_INFO) << "getVaildR Get One R";
        this->R_stat = 1;
    }
    else if (this->R_idx.size() > 1)
    {
        LOG_IF(INFO, this->switch_INFO) << "getVaildR Get Many R:" << this->R_idx.size();
        this->R_stat = 2;
    }
    return this->R_stat;
}

int WMIdentify::selectR()
{
    // 假如没有R，识别失败
    if (this->R_stat == 0)
    {
        LOG_IF(INFO, this->switch_INFO == ON) << "selectR failed";
        return 0;
    }
    // 假如只有一个R，没有选择的必要，直接成功
    else if (this->R_stat == 1)
    {
        LOG_IF(INFO, this->switch_INFO == ON) << "selectR successful";
        // 如果UI开关打开
        if (this->gp->switch_UI == ON)
        {
            // 遍历idx里面每一个索引(其实只有一个，保证结构一致性，不更改)
            for (int i = 0; i < R_idx.size(); i++)
            {
                // 如果轮廓显示开关打开，画出当前找到的R的轮廓
                if (this->gp->switch_UI_contours == ON)
                    cv::drawContours(this->img, this->R_contours, R_idx[i], cv::Scalar(0, 0, 255), 3);
                // 如果面积显示开关打开，显示出当前找到的R的面积、紧致度、圆度
                if (this->gp->switch_UI_areas == ON)
                {
                    double s_area = cv::contourArea(R_contours[R_idx[i]]);
                    cv::putText(this->img, std::to_string(s_area), cv::minAreaRect(this->R_contours[this->R_idx[i]]).center, 1, 1, cv::Scalar(255, 255, 0));
                    double girth = cv::arcLength(R_contours[R_idx[i]], true);
                    double compactness = (girth * girth) / s_area;
                    double circularity = (4 * Pi * s_area) / (girth * girth);
                    cv::putText(this->img, std::to_string(compactness), cv::minAreaRect(this->R_contours[this->R_idx[i]]).center + cv::Point2f(0, 20), 1, 1, cv::Scalar(255, 255, 0));
                    cv::putText(this->img, std::to_string(circularity), cv::minAreaRect(this->R_contours[this->R_idx[i]]).center + cv::Point2f(0, 40), 1, 1, cv::Scalar(255, 255, 0));
                }
            }
        }
        return 1;
    }
    // 假如有很多R，开始筛选
    else if (this->R_stat == 2)
    {
        // 如果UI开关打开
        if (this->gp->switch_UI == ON)
        {
            // 遍历idx里面每一个索引
            for (int i = 0; i < R_idx.size(); i++)
            {
                // 如果轮廓显示开关打开，画出当前找到的R的轮廓
                if (this->gp->switch_UI_contours == ON)
                    cv::drawContours(this->img, this->R_contours, R_idx[i], cv::Scalar(0, 0, 255), 3);
                // 如果面积显示开关打开，显示出当前找到的R的面积、紧致度、圆度
                if (this->gp->switch_UI_areas == ON)
                {
                    double s_area = cv::contourArea(R_contours[R_idx[i]]);
                    cv::putText(this->img, std::to_string(s_area), cv::minAreaRect(this->R_contours[this->R_idx[i]]).center, 1, 1, cv::Scalar(255, 255, 0));
                    double girth = cv::arcLength(R_contours[R_idx[i]], true);
                    double compactness = (girth * girth) / s_area;
                    double circularity = (4 * Pi * s_area) / (girth * girth);
                    cv::putText(this->img, std::to_string(compactness), cv::minAreaRect(this->R_contours[this->R_idx[i]]).center + cv::Point2f(0, 20), 1, 1, cv::Scalar(255, 255, 0));
                    cv::putText(this->img, std::to_string(circularity), cv::minAreaRect(this->R_contours[this->R_idx[i]]).center + cv::Point2f(0, 40), 1, 1, cv::Scalar(255, 255, 0));
                }
            }
        }
        // 遍历全部可能是R的索引，判断是否在中心区域，假如是，认为它是R，让R_idx首位是它，结束筛选
        // for (int i = 0; i < this->R_idx.size(); i++)
        // {
        //     if (cv::minAreaRect(this->R_contours[this->R_idx[i]]).center.x > this->img.cols * this->gp->R_roi_xl && cv::minAreaRect(R_contours[R_idx[i]]).center.x < this->img.cols * this->gp->R_roi_xr && cv::minAreaRect(R_contours[R_idx[i]]).center.y > this->img.rows * this->gp->R_roi_yb && cv::minAreaRect(R_contours[R_idx[i]]).center.y < this->img.rows * this->gp->R_roi_yt)
        //     {
        //         this->R_idx[0] = this->R_idx[i];
        //         LOG_IF(INFO, this->switch_INFO == ON) << "selectR successful";
        //     }
        // }
        std::sort(R_idx.begin(), R_idx.end(), [this](int &a, int &b)
                  { return CalDistSquare(cv::minAreaRect(R_contours[a]).center, R_emitate) < CalDistSquare(cv::minAreaRect(R_contours[b]).center, R_emitate); });
        if (this->R_idx.size() == 1)
        {
            LOG_IF(INFO, this->switch_INFO == ON) << "selectR successful";
            return 1;
        }
        else if (this->R_idx.size() == 0)
        {
            LOG_IF(INFO, this->switch_INFO == ON) << "selectR failed, no R left";
            return 0;
        }
        else if (this->R_idx.size() > 1)
        {
            LOG_IF(INFO, this->switch_INFO == ON) << "selectR failed, too many R left";
            return 0;
        }
    }
    LOG_IF(INFO, this->switch_INFO == ON) << "selectR failed";
    return 0;
}

void WMIdentify::updateList(double time)
{
#ifndef USEWMNET
    list_stat = 1; //<! 队列更新状态，为了保证每个队列中数据数量均同步且对应，1为没有缺失数据，0为缺失数据
    if (this->wing_center_list.size() >= 2)
    {
        this->wing_center_list.pop_front();
    }
    // 如果寻找装甲板中心点确实找到东西了，列表不为空，则把找到的装甲板中心点坐标压进队列中，否则输出日志，数据不足，同时判断当前识别装甲板模式，以通过不同的方式获取中心点坐标
    // winghat的方法时，winghat_idx与R_contours对应，详情间之前步骤注释
    if (this->winghat_idx.size() > 0 && this->R_contours.size() > 0 && this->get_armor_mode == HIERARCHY)
    {
        this->wing_center_list.emplace_back(cv::minAreaRect(this->R_contours[this->winghat_idx[0]]).center);
    }
    else if (this->wing_idx.size() > 0 && this->floodfill_contours.size() > 0 && this->get_armor_mode == FLOODFILL)
    {
        this->wing_center_list.emplace_back(cv::minAreaRect(this->floodfill_contours[this->wing_idx[0]]).center);
    }
    else
    {
        LOG_IF(ERROR, this->switch_ERROR) << "failed to emplace_back wing_center_list, lack data";
        list_stat = 0;
    }
#ifdef DEBUGMODE
    // 如果获取点信息正常，绘制在img上
    if (this->wing_center_list.size() > 0 && list_stat == 1)
        cv::circle(this->img, this->wing_center_list[this->wing_center_list.size() - 1], 10, cv::Scalar(0, 0, 255), -1);
#endif // DEBUGMODE
    if (this->R_center_list.size() >= 2)
    {
        this->R_center_list.pop_front();
    }
    // 如果寻找R中心点确实找到东西了，列表不为空，则把找到的R中心点坐标压进队列中，否则输出日志，数据不足
    if (this->R_idx.size() > 0 && this->R_contours.size() > 0 && list_stat == 1)
    {
        this->R_center_list.emplace_back(cv::minAreaRect(this->R_contours[this->R_idx[0]]).center);
    }
    else
    {
        LOG_IF(ERROR, this->switch_ERROR) << "failed to emplace_back R_center_list, lack data";
        if (list_stat == 1)
        {
            // 此时armor已经输入了数据，但是R无法输入数据，所以要把armor的数据吐出来，保证数据同步
            this->wing_center_list.pop_back();
        }
        list_stat = 0;
    }
#ifdef DEBUGMODE
    // 如果获取点信息正常，绘制在img上
    if (this->R_center_list.size() > 0 && list_stat == 1)
        cv::circle(this->img, this->R_center_list[this->R_center_list.size() - 1], 10, cv::Scalar(0, 255, 255), -1);
    if (this->get_armor_mode == HIERARCHY)
    {
        cv::imshow("result by hierarchy", img);
    }
    else if (this->get_armor_mode == FLOODFILL)
    {
        cv::imshow("result by floodfill", img);
    }
#endif // DEBUGMODE
#else    //   USEWMNET的else
#ifdef DEBUGMODE
    if (this->R_center_list.size() > 0 && list_stat == 1)
        cv::circle(this->img, this->R_center_list[this->R_center_list.size() - 1], 10, cv::Scalar(0, 255, 255), -1);
    if (this->wing_center_list.size() > 0 && list_stat == 1)
        cv::circle(this->img, this->wing_center_list[this->wing_center_list.size() - 1], 10, cv::Scalar(0, 0, 255), -1);
#endif
#endif
    // 更新时间队列
    if (this->time_list.size() >= this->gp->list_size && gp->gap % gp->gap_control == 0)
    {
        this->time_list.pop_front();
    }
    if (list_stat == 1 && gp->gap % gp->gap_control == 0)
    {
        this->time_list.emplace_back(time);
    }
    // 更新角度队列
    if (this->angle_list.size() >= this->gp->list_size && gp->gap % gp->gap_control == 0)
    {
        this->angle_list.pop_front();
    }
    if (list_stat == 1 && gp->gap % gp->gap_control == 0)
    {
        //this->wing_center_list.back().y*=gp->oval2cirel;
        //std::cout<<"now theta"<<CalAngle(this->wing_center_list.back(), this->R_center_list.back())<<std::endl;
        this->angle_list.emplace_back(CalAngle(this->wing_center_list.back(), this->R_center_list.back()));
    }

    // 更新yaw队列
    if (this->R_yaw_list.size() >= this->gp->list_size)
    {
        this->R_yaw_list.pop_front();
    }
    if (list_stat == 1)
    {
        float u = R_center_list[R_center_list.size() - 1].x;
        float v = R_center_list[R_center_list.size() - 1].y;
        float tan_x = (u - (double)this->img.cols / 2) / this->gp->fx;
        // std::cout<<tan_x<<std::endl;
        float yaw = 0;
        if (abs(tan_x)> 0.0001)
            yaw = atan(tan_x);
        this->R_yaw_list.emplace_back(yaw);
    }
    // 更新角速度队列
    if (this->angle_velocity_list.size() >= this->gp->list_size && gp->gap % gp->gap_control == 0)
    {
        this->angle_velocity_list.pop_front();
    }
    if (angle_list.size() > 1 && time_list.size() > 1 && list_stat == 1 && gp->gap % gp->gap_control == 0)
    {
        // 计算角度变化量
        double dangle = this->angle_list[this->angle_list.size() - 1] - this->angle_list[this->angle_list.size() - 2];
        // 防止数据跳变
        dangle += (abs(dangle) > Pi) ? 2 * Pi * (-dangle / abs(dangle)) : 0;
        // 计算时间变化量
        double dtime = (this->time_list[this->time_list.size() - 1] - this->time_list[this->time_list.size() - 2]);
        // 更新角速度队列,简单检验一下数据，获得扇叶切换时间
        if (abs(dangle / dtime) > 5)
        {
            this->FanChangeTime = time_list.back() * 1000;
            this->time_list.pop_front();
            this->angle_list.pop_front();
            gp->gap--;
        }
        else
        {
            this->angle_velocity_list.emplace_back(dangle / dtime);
// #ifdef DEBUGHIT
//             if (angle_velocity_list.size() >= 2)
//             {

//                 cv::Point2f now_point(time_list[time_list.size() - 1], abs(angle_velocity_list[angle_velocity_list.size() - 1]));
//                 static cv::Point2f first_point(time_list.back(), abs(angle_velocity_list.back()));
//                 cv::Point2f last_point(time_list[time_list.size() - 2], abs(angle_velocity_list[angle_velocity_list.size() - 2]));
//                 now_point.x -= first_point.x;
//                 now_point.x *= 20;
//                 now_point.y = now_point.y * 100;
//                 last_point.x -= first_point.x;
//                 last_point.x *= 20;
//                 last_point.y = last_point.y * 100;
//                 cv::circle(this->data_img, now_point, 21, cv::Scalar(255, 255, 255));
//             }

// #endif
        }

        // 更新旋转方向
        // std::cout<<this->time_list.back()<<std::endl;
        // std::cout<<" "<<this->angle_velocity_list.back()<<std::endl;
        this->direction = 0;
        for (int i = 0; i < angle_velocity_list.size(); i++)
        {
            this->direction += this->angle_velocity_list[i];
        }
    }
    // 更新gap
    if (this->list_stat == 1)
    {
        gp->gap++;
    }
    // 输出日志，更新队列成功
    LOG_IF(INFO, this->switch_INFO == ON) << "updateList successful";
}

std::deque<double> WMIdentify::getTimeList()
{
    return this->time_list;
}

std::deque<double> WMIdentify::getAngleVelocityList()
{
    return this->angle_velocity_list;
}
double WMIdentify::getDirection()
{
    return this->direction;
}
double WMIdentify::getAngleList()
{
    return this->angle_list[angle_list.size() - 1];
}
cv::Point2d WMIdentify::getR_center()
{
    return this->R_center_list[R_center_list.size() - 1];
}
double WMIdentify::getRadius()
{
    return sqrt(CalDistSquare(this->R_center_list[R_center_list.size() - 1], this->wing_center_list[wing_center_list.size() - 1]));
}
double WMIdentify::getYaw()
{
    if (R_yaw_list.size() > 0)
        return this->R_yaw_list[R_yaw_list.size() - 1];
    else
        return 0;
}

int WMIdentify::getListStat()
{
    return this->list_stat;
}
cv::Mat WMIdentify::getImg0()
{
    return this->img_0;
}
cv::Mat WMIdentify::getData_img()
{
    return this->data_img;
}
void WMIdentify::ClearSpeed()
{
    this->angle_velocity_list.clear();
    this->time_list.clear();
}
uint32_t WMIdentify::GetFanChangeTime()
{
    return this->FanChangeTime;
}

void WMIdentify::JudgeClear(Translator translator)
{
    if (translator.message.status % 5 == 0)                     //进入自瞄便清空识别的所有数据
        this->clear();
}


bool WMIdentify::IsVaildR(std::vector<cv::Point> contour, GlobalParam &gp)
{
    // double Pi = 3.1415926;
    double s_area = cv::contourArea(contour);
    double girth = cv::arcLength(contour, true);  //计算轮廓周长
    // if (s_area < 150)
    //     return false;
    if (s_area < gp.s_R_min || s_area > gp.s_R_max)
    {
        LOG_IF(INFO, gp.switch_INFO) << "IsVaildR Successful But Not R,s_area wrong:" << s_area;
        // cout<< "IsVaildR Successful But Not R,s_area wrong :  "<<s_area<<endl;
        return false;
    }
    cv::RotatedRect R_rect = minAreaRect(contour);
    cv::Size2f R_size = R_rect.size;
    float length = R_size.height > R_size.width ? R_size.height : R_size.width; // 将矩形的长边设置为长
    float width = R_size.height < R_size.width ? R_size.height : R_size.width;  // 将矩形的短边设置为宽
    float lw_ratio = length / width;
    float s_ratio = s_area / R_size.area();
    if (lw_ratio < gp.R_ratio_min || lw_ratio > gp.R_ratio_max)  //最小外接矩形长宽比筛选
    {
        LOG_IF(INFO, gp.switch_INFO) << "IsVaildR Successful But Not R,lw_ratio wrong:" << lw_ratio << " and s_area is:" << s_area;
        return false;
    }
    if (s_ratio < gp.s_R_ratio_min || s_ratio > gp.s_R_ratio_max) //最小外接矩形填充率筛选
    {
        LOG_IF(INFO, gp.switch_INFO) << "IsVaildR Successful But Not R,s_ratio wrong:" << s_ratio << " and s_area is:" << s_area;
        return false;
    }
    double compactness = (girth * girth) / s_area;
    double circularity = (4 * Pi * s_area) / (girth * girth);
    if (circularity < gp.R_circularity_min || circularity > gp.R_circularity_max)//形状的紧致度筛选
    {
        LOG_IF(INFO, gp.switch_INFO) << "IsVaildR Successful But Not R,circularity wrong:" << circularity << " and s_area is:" << s_area;
        return false;
    }
    if (compactness < gp.R_compactness_min || compactness > gp.R_compactness_max)//形状的圆形度筛选
    {
        LOG_IF(INFO, gp.switch_INFO) << "IsVaildR Successful But Not R,compactness wrong:" << compactness << " and s_area is:" << s_area;
        return false;
    }
    LOG_IF(INFO, gp.switch_INFO) << "IsVaildR Successful,compactness:" << compactness << " circularity:" << circularity << " s_area:" << s_area;
    return true;
}

double WMIdentify::CalAngle(cv::Point p1, cv::Point center)
{
    // double Pi = 3.1415926;
    if(abs(p1.x-center.x)<0.01)
    {
        if(p1.y>center.y)
            return -Pi/2 ;
        else
            return Pi/2;
    }
   
    cv::Point2f dp = p1 - center;
    // 因为图像中y轴向下，所以需要翻转
    double angle = atan2(-dp.y, dp.x);
    if (angle >= 0)
        return angle;
    else
        return angle + 2 * Pi;
    return 0;
}

double WMIdentify::CalDistSquare(cv::Point2f p1, cv::Point2f p2)
{
    return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y);
}