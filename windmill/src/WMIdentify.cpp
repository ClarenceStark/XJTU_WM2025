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
}

WMIdentify::~WMIdentify()
{

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

}

void WMIdentify::startWMINet(cv::Mat &input_img, Translator &ts)
{

    this->GetImagePoints();

    this->GetR_centerPoint();






    this->updateList((double)ts.message.predict_time / 1000);
          
}

void WMIdentify::GetImagePoints(cv::Mat &input_img)
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
            for(int imagepoints_id = 0, imagepoints_id < 5, imagepoints_id++)
            {
                this->imagePoints[imagepoints_id] = object.apex[imagepoints_id];
            }
            this->list_stat = 1;
        }
    }
}


void WMIdentify::GetR_centerPoint()
{
    this->preprocess();
   
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
}

void WMIdentify::receive_pic(cv::Mat &input_img)
{
    this->img = input_img;

    LOG_IF(INFO, this->switch_INFO) << "receive_pic Successful";
}

void WMIdentify::preprocess()
{
    if (this->gp->switch_gaussian_blur == ON)
    {
        cv::GaussianBlur(this->img, this->img, cv::Size(5, 5), 0.0, 0.0);
    }
    cv::cvtColor(this->img, this->img, cv::COLOR_BGR2HSV);
    cv::inRange(this->img, cv::Scalar(this->gp->hmin, this->gp->smin, this->gp->vmin), cv::Scalar(this->gp->hmax, this->gp->smax, this->gp->vmax), this->binary);

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
        }

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