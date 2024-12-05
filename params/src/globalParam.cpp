#include "globalParam.hpp"
#include <glog/logging.h>
#include <filesystem>
#include <iostream>
void GlobalParam::initGlobalParam(const int color)
{   
    const std::string root_path = std::filesystem::current_path();
    cv::FileStorage fs;

    this ->color = color;

    // 打开CameraConfig配置文件
    fs.open(root_path + "/config/CameraConfig.yaml", cv::FileStorage::READ);
    fs["cam_index"] >> cam_index;
    fs["enable_auto_exp"] >> enable_auto_exp;
    fs["energy_exp_time"] >> energy_exp_time;
    fs["armor_exp_time"] >> armor_exp_time;
    fs["r_balance"] >> r_balance;
    fs["g_balance"] >> g_balance;
    fs["b_balance"] >> b_balance;
    fs["e_r_balance"] >> e_r_balance;
    fs["e_g_balance"] >> e_g_balance;
    fs["e_b_balance"] >> e_b_balance;
    fs["enable_auto_gain"] >> enable_auto_gain;
    fs["gain"] >> gain;
    fs["gamma_value"] >> gamma_value;
    fs["trigger_activation"] >> trigger_activation;
    fs["frame_rate"] >> frame_rate;
    fs["enable_trigger"] >> enable_trigger;
    fs["trigger_source"] >> trigger_source;
    fs["cx"] >> cx;  // cx
    fs["cy"] >> cy;  // cy
    fs["fx"] >> fx;  // fx
    fs["fy"] >> fy;  // fy
    fs["k1"] >> k1;
    fs["k2"] >> k2;
    fs["k3"] >> k3;
    fs["p1"] >> p1;
    fs["p2"] >> p2;
    fs.release();
    
    // 打开AimautoConfig配置文件
    fs.open(root_path + "/config/AimautoConfig.yaml", cv::FileStorage::READ);
    fs["cost_threshold"] >> cost_threshold;
    fs["max_lost_frame"] >> max_lost_frame;
    // 卡尔曼滤波相关参数
    fs["s2qxyz"] >> s2qxyz;              // 位置转移噪声
    fs["s2qyaw"] >> s2qyaw;              // 角度转移噪声
    fs["s2qr"] >> s2qr;                  // 半径转移噪声
    fs["r_xy_factor"] >> r_xy_factor;     
    fs["r_z"] >> r_z;     
    fs["r_yaw"] >> r_yaw;
    fs.release();

    // 打开DetectionConfig配置文件
    fs.open(root_path + "/config/DetectionConfig.yaml", cv::FileStorage::READ);
    fs["min_ratio"] >> min_ratio;
    fs["max_ratio"] >> max_ratio;
    fs["max_angle_l"] >> max_angle_l;
    fs["min_light_ratio"] >> min_light_ratio;
    fs["min_small_center_distance"] >> min_small_center_distance;
    fs["max_small_center_distance"] >> max_small_center_distance;
    fs["min_large_center_distance"] >> min_large_center_distance;
    fs["max_large_center_distance"] >> max_large_center_distance;
    fs["max_angle_a"] >> max_angle_a;
    fs["num_threshold"] >> num_threshold;
    fs["blue_threshold"] >> blue_threshold;
    fs["red_threshold"] >> red_threshold;
    fs.release();
    
    LOG_IF(INFO, switch_INFO) << "initGlobalParam Successful";
}

void GlobalParam::saveGlobalParam()
{
    cv::FileStorage fs;

    // 打开CameraConfig配置文件以写入参数
    fs.open("../config/CameraConfig.yaml", cv::FileStorage::WRITE);
    fs << "cam_index" << cam_index;
    fs << "enable_auto_exp" << enable_auto_exp;
    fs << "energy_exp_time" << energy_exp_time;
    fs << "armor_exp_time" << armor_exp_time;
    fs << "r_balance" << r_balance;
    fs << "g_balance" << g_balance;
    fs << "b_balance" << b_balance;
    fs << "enable_auto_gain" << enable_auto_gain;
    fs << "gain" << gain;
    fs << "gamma_value" << gamma_value;
    fs << "trigger_activation" << trigger_activation;
    fs << "frame_rate" << frame_rate;
    fs << "enable_trigger" << enable_trigger;
    fs << "trigger_source" << trigger_source;
    fs << "cx" << cx;  // cx
    fs << "cy" << cy;  // cy
    fs << "fx" << fx;  // fx
    fs << "fy" << fy;  // fy
    fs << "k1" << k1;
    fs << "k2" << k2;
    fs << "k3" << k3;
    fs << "p1" << p1;
    fs << "p2" << p2;
    fs.release();

    // 打开AimautoConfig配置文件以写入参数
    fs.open("../config/AimautoConfig.yaml", cv::FileStorage::WRITE);
    fs << "cost_threshold" << cost_threshold;
    fs << "max_lost_frame" << max_lost_frame;
    // 卡尔曼滤波相关参数
    fs << "s2qxyz" << s2qxyz;                // 位置转移噪声
    fs << "s2qyaw" << s2qyaw;                // 角度转移噪声
    fs << "s2qr" << s2qr;                    // 半径转移噪声
    fs << "r_xy_factor" << r_xy_factor;     
    fs << "r_z" << r_z;    
    fs << "r_yaw" << r_yaw;    
    fs.release();              

    // 打开DetectionConfig配置文件以写入参数
    fs.open("../config/DetectionConfig.yaml", cv::FileStorage::WRITE);
    fs << "min_ratio" << min_ratio;
    fs << "max_ratio" << max_ratio;
    fs << "max_angle_l" << max_angle_l;
    fs << "min_light_ratio" << min_light_ratio;
    fs << "min_small_center_distance" << min_small_center_distance;
    fs << "max_small_center_distance" << max_small_center_distance;
    fs << "min_large_center_distance" << min_large_center_distance;
    fs << "max_large_center_distance" << max_large_center_distance;
    fs << "max_angle_a" << max_angle_a;
    fs << "num_threshold" << num_threshold;
    fs << "blue_threshold" << blue_threshold;
    fs << "red_threshold" << red_threshold;
    fs.release();

    LOG_IF(INFO, switch_INFO) << "saveGlobalParam Successful";
}