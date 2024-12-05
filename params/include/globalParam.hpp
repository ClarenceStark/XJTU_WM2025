/**
 * @file globalParam.hpp
 * @author axi404 (3406402603@qq.com)
 * @brief 参数列表文件，由结构体与枚举组成，暂时仅包括打符识别需要的参数
 * @version 0.1
 * @date 2022-12-30
 *
 * @copyright Copyright (c) 2022
 *
 */
#if !defined(__GLOBALPARAM_HPP)
#define __GLOBALPARAM_HPP
#include "Eigen/Eigen"
#include "MvCameraControl.h"
#include "deque"
#include "opencv2/core.hpp"
#include <cstdint>
#include <opencv2/core/types.hpp>
#pragma pack(1)
typedef struct
{

    uint8_t head; // 0x71
    // 电控发送的信息
    float yaw;             // 当前车云台的yaw角，单位为弧度制
    float pitch;           // 当前车云台的pitch角，单位为弧度制
    uint8_t status;        // 状态位，/5==0自己为红色，/5==0自己为蓝色，%5==0为自瞄，%5==1为小符，%5==3为大符
    uint16_t bullet_v;     // 上一次发射的弹速，单位为米每秒
    uint8_t armor_flag;    // 敌方3-5号车是否为大装甲板(即平衡车)，为二进制转十进制，如345全为平衡，为111，输入7，只有3为平衡，为100，输入4
    uint32_t predict_time; 
    float x_a;             // 装甲板在世界坐标系(云台pitch、yaw为0时的相机坐标系)下的x坐标
    float y_a;             // 装甲板在世界坐标系下的y坐标
    float z_a;             // 装甲板在世界坐标系下的z坐标
    float x_c;             // 车中心点在世界坐标系下的x坐标
    float y_c;             // 车中心点在世界坐标系下的y坐标
    float z_c;             // 车中心点在世界坐标系下的z坐标
    float yaw_a;           // 装甲板与车中心点连线与世界坐标系x轴夹角，装甲板正对时为0，车顺时针旋转为负，反之为正，范围大约在-pi/2至pi/2，
    float vx_c;            // 车中心点在世界坐标系下的x速度
    float vy_c;            // 车中心点在世界坐标系下的y速度
    float vz_c;            // 车中心点在世界坐标系下的z速度
    float vyaw_a;          // 如上定义的角，车旋转的角速度
    uint16_t crc;
    uint8_t tail; // 0x4C

} MessData;
#pragma pack()
typedef union
{
    MessData message;
    char data[64];
} Translator;

enum class ArmorType { SMALL, LARGE, INVALID };
const std::string ARMOR_TYPE_STR[3] = {"small", "large", "invalid"};

struct Light : public cv::RotatedRect
{
  Light() = default;
  explicit Light(cv::RotatedRect box) : cv::RotatedRect(box)
  {
    cv::Point2f p[4];
    box.points(p);
    std::sort(p, p + 4, [](const cv::Point2f & a, const cv::Point2f & b) { return a.y < b.y; });
    top = (p[0] + p[1]) / 2;
    bottom = (p[2] + p[3]) / 2;

    length = cv::norm(top - bottom);
    width = cv::norm(p[0] - p[1]);

    tilt_angle = std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y));
    tilt_angle = tilt_angle / CV_PI * 180;
  }

  int color;
  cv::Point2f top, bottom;
  double length;
  double width;
  float tilt_angle;
};

struct UnsolvedArmor
{
  UnsolvedArmor() = default;
  UnsolvedArmor(const Light & l1, const Light & l2)
  {
    if (l1.center.x < l2.center.x) {
      left_light = l1, right_light = l2;
    } else {
      left_light = l2, right_light = l1;
    }
    center = (left_light.center + right_light.center) / 2;
  }

  // Light pairs part
  Light left_light, right_light;
  cv::Point2f center;
  ArmorType type;

  // Number part
  cv::Mat number_img;
  std::string number;
  float confidence;
  std::string classfication_result;
};
  // namespace rm_auto_aim

struct Armor
{
    int color;
    int type;
    cv::Point3f center;
    cv::Point3f angle;
    cv::Point2f apex[4];
    double distance_to_image_center;
    Eigen::Vector3d position;
    double yaw;
};
struct Armors
{
    std::deque<Armor> armors;
};
struct ArmorObject
{
    cv::Point2f apex[4];
    cv::Rect_<float> rect;
    int cls;
    int color;
    int area;
    float prob;
    std::vector<cv::Point2f> pts;
};
struct WMObject
{
    cv::Point2f apex[4];
    cv::Rect_<float> rect;
    int cls;
    int color;
    int area;
    float prob;
    std::vector<cv::Point2f> pts;
};
enum COLOR
{
    RED = 0,
    BLUE = 1
};
enum TARGET
{
    SMALL_ARMOR = 0,
    BIG_ARMOR = 1
};

enum ATTACK_MODE
{
    ENERGY = 0,
    ARMOR = 1
};

enum SWITCH
{
    OFF = 0,
    ON = 1
};

enum GETARMORMODE
{
    HIERARCHY = 0,
    FLOODFILL = 1
};

/**
 * @brief 全局参数结构体，用于保存所有可以更改的参数或贯穿全局的状态量，请在对应yaml配置文件内修改其具体值，而不要在此文件内修改
 *
 *
 */
struct GlobalParam
{
    //==============================全局状态量部分==============================//
    int color = BLUE;                       // 当前颜色
    // int get_armor_mode = FLOODFILL;      // 当前获取armor中心点方法
    // 调试信息，INFO等级的日志是否输出
    int switch_INFO = ON;                   // 调试信息，ERROR等级的日志是否输出
    int switch_ERROR = ON;

    //============================信息管理参数================================//
    int message_hold_threshold = 5;
    float fake_pitch = 0.0F;
    float fake_yaw = 0.0F;
    float fake_bullet_v = 25.0F;
    uint8_t fake_status = 5;
    float fake_now_time = 0;
    float fake_predict_time = 0;

    //==============================相机部分==============================//
    int attack_mode = ARMOR;
    // 当前使用的相机序号，只连接一个相机时为0，多个相机在取流时可通过cam_index选择设备列表中对应顺序的相机
    int cam_index = 0;
    //===曝光时间===//
    MV_CAM_EXPOSURE_AUTO_MODE enable_auto_exp = MV_EXPOSURE_AUTO_MODE_OFF;
    float energy_exp_time = 200.0F;         // 能量机关曝光时间
    float armor_exp_time = 290.0F;          // 装甲板曝光时间
    float height = 1080;
    float width = 1440;
    //===白平衡===/
    // 默认为目标为红色的白平衡
    int r_balance = 1500;   // 红色通道
    int g_balance = 1024;   // 绿色通道
    int b_balance = 4000;   // 蓝色通道
    int e_r_balance = 1500; // 红色通道
    int e_g_balance = 1024; // 绿色通道
    int e_b_balance = 4000; // 蓝色通道
    //===以下参数重点参与实际帧率的调节===//
    unsigned int pixel_format = PixelType_Gvsp_BayerRG8;    // 图像格式，设置为Bayer RG8，更多图像格式可前往MVS的SDK中寻找
    // 在经过测试之后，包括在官方SDK中没有调整Acquisition Frame Rate Control Enable的参数，不过在MVS软件中调试时此选项关闭后依然可以达到期望帧率
    //===其他参数===//
    MV_CAM_GAIN_MODE enable_auto_gain = MV_GAIN_MODE_OFF;   // 自动相机增益使能
    float gain = 17.0F;             // 相机增益值
    float gamma_value = 0.7F;       // 相机伽马值，只有在伽马修正开启后有效
    int trigger_activation = 0;     // 触发方式，从0至3依次为触发上升沿、下降沿、高电平、低电平
    float frame_rate = 180.0F;      // 设置帧率，仅在不设置外触发时起效
    MV_CAM_TRIGGER_MODE enable_trigger = MV_TRIGGER_MODE_OFF;           // 触发模式，ON为外触发，OFF为内触发
    MV_CAM_TRIGGER_SOURCE trigger_source = MV_TRIGGER_SOURCE_LINE0;     // 触发源
    // 内参矩阵参数
    double cx = 697.7909321;      //<! cx
    double cy = 559.5145959;      //<! cy
    double fx = 2349.641948; //<! fx
    double fy = 2351.226326; //<! fy
    // 畸变矩阵参数
    double k1 = -0.041595291261727;
    double k2 = 0.090691694332708;
    double k3 = 0;
    double p1 = 0;
    double p2 = 0;

    //============================装甲板识别相关参数============================//
    double min_ratio = 0.02;
    double max_ratio = 0.75;
    double max_angle_l = 60.0;
    double min_light_ratio = 0.5;
    double min_small_center_distance = 0.8;
    double max_small_center_distance = 2.9;
    double min_large_center_distance = 3.2;
    double max_large_center_distance = 6.0;
    double max_angle_a = 60.0;
    double num_threshold = 0.8;

    int blue_threshold = 65;
    int red_threshold = 70;

    
    //============================自瞄相关参数===============================//
    double threshold_low = 250.0;
    int armorStat = 0;
    bool isBigArmor[6] = {0, 1, 0, 0, 0, 0};
    // double max_match_distance = 0.5;
    // double max_match_yaw_diff = 0.75;
    double cost_threshold = 1000;
    double max_lost_frame = 10;
    // 卡尔曼滤波相关参数
    double s2qxyz = 250.0;           // 位置转移噪声
    double s2qyaw = 90.0;            // 角度转移噪声
    double s2qr = 250.0;             // 半径转移噪声
    double r_xy_factor = 0.032;  
    double r_z = 1e-7; 
    double r_yaw = 0.016;  

    //===新加的===//
    // int realy_mid = 720;
    // double camera2shootBias = 0.0;
    // double pzy = 0.0;

    // int binary_thres = 100;

    // GlobalParam(){
    //     initGlobalParam(BLUE);
    // }

    void initGlobalParam(const int color);
    void saveGlobalParam();
};

#endif // __GLOBALPARAM_HPP