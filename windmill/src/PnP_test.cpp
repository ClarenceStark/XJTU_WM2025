#include "MvCameraControl.h"
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <unistd.h>

#include <iostream>
#include <vector>

using namespace cv;
using namespace std;

// void drawAxis(cv::Mat &image, cv::Mat &rvec, cv::Mat &tvec,
//               cv::Mat &cameraMatrix, cv::Mat &distCoeffs) {
//   // 定义坐标轴的3D点 (单位: 米)
//   std::vector<cv::Point3f> axisPoints;
//   axisPoints.push_back(cv::Point3f(0.02, 0.035, 0));   // 原点
//   axisPoints.push_back(cv::Point3f(0.12, 0.035, 0)); // x轴 (红色)
//   axisPoints.push_back(cv::Point3f(0.02, 0.135, 0)); // y轴 (绿色)
//   axisPoints.push_back(cv::Point3f(0.02, 0.035, -0.01)); // z轴 (蓝色)

//   // 将3D坐标轴的点投影到图像平面上
//   std::vector<cv::Point2f> imagePoints;
//   cv::projectPoints(axisPoints, rvec, tvec, cameraMatrix, distCoeffs,
//                     imagePoints);

//   // 绘制坐标轴 (红色: x轴，绿色: y轴，蓝色: z轴)
//   cv::line(image, imagePoints[0], imagePoints[1], cv::Scalar(0, 0, 255),
//            4); // x轴
//   cv::line(image, imagePoints[0], imagePoints[2], cv::Scalar(0, 255, 0),
//            4); // y轴
//   cv::line(image, imagePoints[0], imagePoints[3], cv::Scalar(255, 0, 0),
//            4); // z轴
//            int fontFace = cv::FONT_HERSHEY_SIMPLEX;
//     double fontScale = 0.5;
//            cv::putText(image, "x", imagePoints[1], fontFace, fontScale, cv::Scalar(0, 0, 255), 4);
//     cv::putText(image, "y", imagePoints[2], fontFace, fontScale, cv::Scalar(0, 255, 0), 4);
//     cv::putText(image, "z", imagePoints[3], fontFace, fontScale, cv::Scalar(255, 0, 0), 4);
// }
void drawAxis(cv::Mat &image, cv::Mat &rvec, cv::Mat &tvec,
              cv::Mat &cameraMatrix, cv::Mat &distCoeffs) {
    // 定义坐标轴的3D点 (单位: 米)
    std::vector<cv::Point3f> axisPoints;
    axisPoints.push_back(cv::Point3f(0.0165, 0.0165, 0));     // 原点
    axisPoints.push_back(cv::Point3f(0.0565, 0.0165, 0));     // x轴终点
    axisPoints.push_back(cv::Point3f(0.0165, 0.0565, 0));     // y轴终点

    // 计算相机z轴与世界z轴的夹角
    cv::Mat R;
    cv::Rodrigues(rvec, R); // 将旋转向量转换为旋转矩阵
    cv::Vec3d cameraZAxis = R.col(2); // 相机坐标系下的z轴方向向量
    cv::Vec3d worldZAxis(0, 0, 1);    // 世界坐标系下的z轴方向向量

    double cosTheta = cameraZAxis.dot(worldZAxis) / (cv::norm(cameraZAxis) * cv::norm(worldZAxis));
    double zLength = 0.2 * (1.0 - cosTheta); // 根据夹角动态调整z轴长度
    cout << cosTheta << endl;
    // 添加动态调整后的z轴终点
    axisPoints.push_back(cv::Point3f(0.0165, 0.0165, -zLength)); // z轴终点（根据夹角动态调整）

    // 将3D坐标轴的点投影到图像平面上
    std::vector<cv::Point2f> imagePoints;
    cv::projectPoints(axisPoints, rvec, tvec, cameraMatrix, distCoeffs, imagePoints);

    // 绘制坐标轴 (红色: x轴，绿色: y轴，蓝色: z轴)
    cv::line(image, imagePoints[0], imagePoints[1], cv::Scalar(0, 0, 255), 10); // x轴
    cv::line(image, imagePoints[0], imagePoints[2], cv::Scalar(0, 255, 0), 10); // y轴
    cv::line(image, imagePoints[0], imagePoints[3], cv::Scalar(255, 0, 0), 10); // z轴

    // 标注轴号
    int fontFace = cv::FONT_HERSHEY_SIMPLEX;
    double fontScale = 3;
    cv::putText(image, "x", imagePoints[1], fontFace, fontScale, cv::Scalar(0, 0, 255), 4);
    cv::putText(image, "y", imagePoints[2], fontFace, fontScale, cv::Scalar(0, 255, 0), 4);
    cv::putText(image, "z", imagePoints[3], fontFace, fontScale, cv::Scalar(255, 0, 0), 4);
}

// 添加一个函数，用于对角点进行排序
// void sortCorners(std::vector<cv::Point2f>& corners) {
//     // 计算质心
//     cv::Point2f center(0, 0);
//     for (const auto& corner : corners)
//         center += corner;
//     center *= (1.0 / corners.size());

//     // 计算角点相对于质心的角度
//     std::vector<std::pair<double, cv::Point2f>> angles;
//     for (const auto& corner : corners) {
//         double angle = atan2(corner.y - center.y, corner.x - center.x);
//         angles.push_back(std::make_pair(angle, corner));
//     }

//     // 按角度排序
//     std::sort(angles.begin(), angles.end(),
//               [](const std::pair<double, cv::Point2f>& a, const std::pair<double, cv::Point2f>& b) {
//                   return a.first < b.first;
//               });

//     // 更新排序后的角点
//     for (int i = 0; i < corners.size(); ++i) {
//         corners[i] = angles[i].second;
//     }
// }
void sortCorners(std::vector<cv::Point2f>& corners) {
    // 计算中心点
    cv::Point2f center(0, 0);
    for (const auto& corner : corners)
        center += corner;
    center *= (1.0 / corners.size());

    std::vector<cv::Point2f> top, bottom;
    for (const auto& corner : corners) {
        if (corner.y < center.y)
            top.push_back(corner);
        else
            bottom.push_back(corner);
    }

    std::vector<cv::Point2f> ordered(4);
    if (top.size() == 2 && bottom.size() == 2) {
        // 左上和右上
        if (top[0].x < top[1].x) {
            ordered[0] = top[0]; // 左上
            ordered[1] = top[1]; // 右上
        } else {
            ordered[0] = top[1];
            ordered[1] = top[0];
        }
        // 左下和右下
        if (bottom[0].x < bottom[1].x) {
            ordered[3] = bottom[0]; // 左下
            ordered[2] = bottom[1]; // 右下
        } else {
            ordered[3] = bottom[1];
            ordered[2] = bottom[0];
        }
    }
    corners = ordered;
}

void processFrame(cv::Mat &frame, cv::Mat &cameraMatrix, cv::Mat &distCoeffs) {
    double  PI = 3.14159265358979323846;
  

    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    cv::Mat binary;
    cv::threshold(gray, binary, 200, 255, cv::THRESH_BINARY);
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(binary, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    // 遍历所有轮廓，寻找具有4个角点的轮廓（矩形）

    for (int i = 0; i < contours.size(); i++) {
      std::vector<cv::Point> approx;
      //cv::approxPolyDP(contours[i], approx, 13, true);
      double perimeter = cv::arcLength(contours[i], true);
      cv::approxPolyDP(contours[i], approx, 0.02 * perimeter, true);


      if (approx.size() == 4 && cv::isContourConvex(approx)) {
        // 检测到矩形，计算它的角点
        std::vector<cv::Point2f> imagePoints;
        drawContours(frame, contours, i, Scalar(0, 255, 255), 2);
        for (int i = 0; i < 4; i++) {
          imagePoints.push_back(approx[i]);
        }
        cv::cornerSubPix(gray, imagePoints, cv::Size(5, 5), cv::Size(-1, -1),
                              cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
        sortCorners(imagePoints);

        // 世界坐标系下的矩形的4个角点
        std::vector<cv::Point3f> objectPoints;
        objectPoints.push_back(cv::Point3f(0, 0, 0));       // 左下角
        objectPoints.push_back(cv::Point3f(0.033, 0, 0));    // 右下角
        objectPoints.push_back(cv::Point3f(0.033, 0.033, 0)); // 右上角
        objectPoints.push_back(cv::Point3f(0, 0.033, 0));    // 左上角

        // PnP解算，得到旋转向量和平移向量
        cv::Mat rvec, tvec;
        cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec,
                    tvec);
        
        cv::Mat rotation_matrix;
        double theta = rvec.at<double>(2);
        cv::Rodrigues(rvec, rotation_matrix);
        
        double yaw = - std::atan2(rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(0, 0));
        // double pitch = - std::atan2(rotation_matrix.at<double>(1, 0), rotation_matrix.at<double>(0, 0)) * 2;
        double pitch = - std::atan2(rotation_matrix.at<double>(1, 0), rotation_matrix.at<double>(0, 0));
        double roll = std::atan2(rotation_matrix.at<double>(2, 1), rotation_matrix.at<double>(2, 2));

        double pitch1 = atan2(-rotation_matrix.at<double>(2, 0), sqrt(rotation_matrix.at<double>(0, 0) * rotation_matrix.at<double>(0, 0) + rotation_matrix.at<double>(1, 0) * rotation_matrix.at<double>(1, 0))); 
            // 计算 yaw (绕Z轴的旋转) 
        double yaw1 = atan2(rotation_matrix.at<double>(1, 0), rotation_matrix.at<double>(0, 0));
            // 计算 roll (绕X轴的旋转) 
        double roll1 = atan2(rotation_matrix.at<double>(2, 1), rotation_matrix.at<double>(2, 2));

        cv::Mat Cw = -rotation_matrix.t() * tvec;
        // 计算从相机光心到世界原点的向量（V）
        cv::Mat V = -Cw; // 因为世界原点在 (0, 0, 0)

        // 将向量 V 投影到 xoz 平面（V_xz）
        double Vx = V.at<double>(0, 0);
        double Vz = V.at<double>(2, 0);

        // 计算 α（alpha）
        double alpha = atan2(Vx, Vz); // V_xz 与世界z轴之间的夹角

        // 获取相机的 z轴在世界坐标系中的表示（Zc_w）
        cv::Mat Zc_w = rotation_matrix.t().col(2); // rot_mat.t() 的第三列

        // 将 Zc_w 投影到 xoz 平面（Zc_w_xz）
        double Zx = Zc_w.at<double>(0, 0);
        double Zz = Zc_w.at<double>(2, 0);

        // 计算 β（beta）
        double beta = atan2(Zx, Zz); // Zc_w_xz 与世界z轴之间的夹角

        // 计算 φ（phi）
        // 计算点积和模长
        double dot_product = Vx * Zx + Vz * Zz;
        double mag_V = sqrt(Vx * Vx + Vz * Vz);
        double mag_Z = sqrt(Zx * Zx + Zz * Zz);

        // 使用余弦公式计算 φ
        double phi = acos(dot_product / (mag_V * mag_Z));

        alpha = alpha * (180.0 / CV_PI);
        beta = beta * (180.0 / CV_PI);
        phi = phi * (180.0 / CV_PI);
        
        std::cout << "alpha: " << alpha << "\t beta: "<< beta << "\t phi: "<< phi << std::endl;
        std::cout << "pitch: " << pitch / PI * 180 << "\t roll: "<< roll / PI * 180 << "\t yaw: "<< yaw / PI * 180  << std::endl;
        std::cout << "pitch1: " << pitch1 / PI * 180 << "\t roll1: "<< roll1 / PI * 180 << "\t yaw1: "<< yaw1 / PI * 180  << std::endl;

        // 在图像上绘制矩形
        for (int i = 0; i < 4; i++) {
          cv::line(frame, approx[i], approx[(i + 1) % 4], cv::Scalar(0, 255, 255),
                  2);
        }

        // 可视化世界坐标系
        drawAxis(frame, rvec, tvec, cameraMatrix, distCoeffs);
        cv::imshow("frame", binary);
        waitKey(0);
        // 找到第一个矩形后退出循环
        break;
      }
    }
}

// ch:等待按键输入 | en:Wait for key press
void WaitForKeyPress(void) {
  printf("Press enter to continue...\n");
  getchar();
}

bool PrintDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo) {
  if (NULL == pstMVDevInfo) {
    printf("The Pointer of pstMVDevInfo is NULL!\n");
    return false;
  }
  if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE) {
    int nIp1 =
        ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
    int nIp2 =
        ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
    int nIp3 =
        ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
    int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);
    printf("CurrentIp: %d.%d.%d.%d\n", nIp1, nIp2, nIp3, nIp4);
    printf("UserDefinedName: %s\n\n",
           pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
  } else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE) {
    printf("UserDefinedName: %s\n",
           pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
    printf("Serial Number: %s\n",
           pstMVDevInfo->SpecialInfo.stUsb3VInfo.chSerialNumber);
    printf("Device Number: %d\n\n",
           pstMVDevInfo->SpecialInfo.stUsb3VInfo.nDeviceNumber);
  } else {
    printf("Not support.\n");
  }
  return true;
}

bool IsColor(MvGvspPixelType enType) {
  switch (enType) {
  case PixelType_Gvsp_BGR8_Packed:
  case PixelType_Gvsp_YUV422_Packed:
  case PixelType_Gvsp_YUV422_YUYV_Packed:
  case PixelType_Gvsp_BayerGR8:
  case PixelType_Gvsp_BayerRG8:
  case PixelType_Gvsp_BayerGB8:
  case PixelType_Gvsp_BayerBG8:
    return true;
  default:
    return false;
  }
}

bool IsMono(MvGvspPixelType enType) {
  switch (enType) {
  case PixelType_Gvsp_Mono8:
  case PixelType_Gvsp_Mono10:
  case PixelType_Gvsp_Mono10_Packed:
  case PixelType_Gvsp_Mono12:
  case PixelType_Gvsp_Mono12_Packed:
    return true;
  default:
    return false;
  }
}

int main() {
  int nRet = MV_OK;
  void *handle = NULL;
  unsigned char *pConvertData = NULL;
  unsigned int nConvertDataSize = 0;
  cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 2383.72, 0, 728.1911, 0,
                          2383.836, 591.0267, 0, 0, 1);
  cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F); // 假设无畸变

  do {
    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (MV_OK != nRet) {
      printf("Enum Devices fail! nRet [0x%x]\n", nRet);
      break;
    }

    if (stDeviceList.nDeviceNum > 0) {
      for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++) {
        printf("[device %d]:\n", i);
        MV_CC_DEVICE_INFO *pDeviceInfo = stDeviceList.pDeviceInfo[i];
        if (NULL == pDeviceInfo) {
          break;
        }
        PrintDeviceInfo(pDeviceInfo);
      }
    } else {
      printf("Find No Devices!\n");
      break;
    }

    printf("Please Input camera index(0-%d):", stDeviceList.nDeviceNum - 1);
    unsigned int nIndex = 0;
    scanf("%d", &nIndex);

    if (nIndex >= stDeviceList.nDeviceNum) {
      printf("Input error!\n");
      break;
    }

    nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
    if (MV_OK != nRet) {
      printf("Create Handle fail! nRet [0x%x]\n", nRet);
      break;
    }

    nRet = MV_CC_OpenDevice(handle, 1, 0);
    if (MV_OK != nRet) {
      printf("Open Device fail! nRet [0x%x]\n", nRet);
      break;
    }

    if (stDeviceList.pDeviceInfo[nIndex]->nTLayerType == MV_GIGE_DEVICE) {
      int nPacketSize = MV_CC_GetOptimalPacketSize(handle);
      if (nPacketSize > 0) {
        nRet = MV_CC_SetIntValue(handle, "GevSCPSPacketSize", nPacketSize);
        if (nRet != MV_OK) {
          printf("Warning: Set Packet Size fail nRet [0x%x]!", nRet);
        }
      } else {
        printf("Warning: Get Packet Size fail nRet [0x%x]!", nPacketSize);
      }
    }

    nRet = MV_CC_SetEnumValue(handle, "TriggerMode", MV_TRIGGER_MODE_OFF);
    if (MV_OK != nRet) {
      printf("Set Trigger Mode fail! nRet [0x%x]", nRet);
      break;
    }

    nRet = MV_CC_SetEnumValue(handle, "ExposureMode", 0); // 0：Timed
    nRet = MV_CC_SetFloatValue(handle, "ExposureTime", 10000);
    if (MV_OK != nRet) {
      printf("Set ExposureTime fail nRet [0xd%]\n", nRet);
    }
    // nRet = MV_CC_SetEnumValue(handle, "ExposureAuto", 2);
    // if (MV_OK != nRet) {
    //   printf("Set GammaSelector fail! nRet [0x%x]\n", nRet);
    // }
    nRet = MV_CC_StartGrabbing(handle);
    if (MV_OK != nRet) {
      printf("Start Grabbing fail! nRet [0x%x]\n", nRet);
      break;
    }

    nRet = MV_CC_SetBoolValue(handle, "GammaEnable", true);
    if (MV_OK != nRet) {
      printf("Set GammaEnable fail! nRet [0x%x]\n", nRet);
    }
    // 2.设置gamma类型，user：1，sRGB：2
    nRet = MV_CC_SetEnumValue(handle, "GammaSelector", 2);
    if (MV_OK != nRet) {
      printf("Set GammaSelector fail! nRet [0x%x]\n", nRet);
    }
    // nRet = MV_CC_SetEnumValue(handle, "GainAuto", 2);
    // if (MV_OK != nRet) {
    //   printf("Set GammaSelector fail! nRet [0x%x]\n", nRet);
    // }
    // nRet = MV_CC_SetFloatValue(handle, "Gain", 10);
    // if (MV_OK != nRet) {
    //   printf("Set Gain fail! nRet [0x%x]\n", nRet);
    // }
    // 3.设置gamma值，推荐范围0.5-2，1为线性拉伸
    // nRet = MV_CC_SetFloatValue(handle, "Gamma", 1);
    // if (MV_OK != nRet) {
    //   printf("Set Gamma failed! nRet [0xd%]\n", nRet);
    // }

    // nRet = MV_CC_SetEnumValue(handle, "BalanceWhiteAuto", 1);
    // if (MV_OK != nRet) {
    //   printf("Set BalanceWhiteAuto  fail! nRet [0x%x]\n", nRet);
    // }
    // // 取流之后，自动白平衡采集一段时间，相机自动调整
    // // 调整完毕后后，关闭自动白平衡，即可
    // sleep(2000);
    // // 关闭自动白平衡
    // nRet = MV_CC_SetEnumValue(handle, "BalanceWhiteAuto", 0);
    // if (MV_OK != nRet) {
    //   printf("Set BalanceWhiteAuto  fail! nRet [0x%x]\n", nRet);
    // }

    while (true) {
      MV_FRAME_OUT stImageInfo = {0};
      nRet = MV_CC_GetImageBuffer(handle, &stImageInfo, 1000);
      if (nRet == MV_OK) {
        printf("Get One Frame: Width[%d], Height[%d], nFrameNum[%d]\n",
               stImageInfo.stFrameInfo.nWidth, stImageInfo.stFrameInfo.nHeight,
               stImageInfo.stFrameInfo.nFrameNum);

        MvGvspPixelType enDstPixelType = PixelType_Gvsp_Undefined;
        unsigned int nChannelNum = 0;

        if (IsColor(stImageInfo.stFrameInfo.enPixelType)) {
          nChannelNum = 3;
          enDstPixelType = PixelType_Gvsp_RGB8_Packed;
        } else if (IsMono(stImageInfo.stFrameInfo.enPixelType)) {
          nChannelNum = 1;
          enDstPixelType = PixelType_Gvsp_Mono8;
        }

        if (enDstPixelType != PixelType_Gvsp_Undefined) {
          pConvertData = (unsigned char *)malloc(
              stImageInfo.stFrameInfo.nWidth * stImageInfo.stFrameInfo.nHeight *
              nChannelNum);
          if (NULL == pConvertData) {
            printf("malloc pConvertData fail!\n");
            nRet = MV_E_RESOURCE;
            break;
          }
          nConvertDataSize = stImageInfo.stFrameInfo.nWidth *
                             stImageInfo.stFrameInfo.nHeight * nChannelNum;

          MV_CC_PIXEL_CONVERT_PARAM stConvertParam = {0};
          stConvertParam.nWidth = stImageInfo.stFrameInfo.nWidth;
          stConvertParam.nHeight = stImageInfo.stFrameInfo.nHeight;
          stConvertParam.pSrcData = stImageInfo.pBufAddr;
          stConvertParam.nSrcDataLen = stImageInfo.stFrameInfo.nFrameLen;
          stConvertParam.enSrcPixelType = stImageInfo.stFrameInfo.enPixelType;
          stConvertParam.enDstPixelType = enDstPixelType;
          stConvertParam.pDstBuffer = pConvertData;
          stConvertParam.nDstBufferSize = nConvertDataSize;
          nRet = MV_CC_ConvertPixelType(handle, &stConvertParam);
          if (MV_OK != nRet) {
            printf("Convert Pixel Type fail! nRet [0x%x]\n", nRet);
            break;
          }

          // 使用 OpenCV 展示图像
          cv::Mat image;
          if (nChannelNum == 3) {
            image =
                cv::Mat(stImageInfo.stFrameInfo.nHeight,
                        stImageInfo.stFrameInfo.nWidth, CV_8UC3, pConvertData);
            cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
            processFrame(image, cameraMatrix, distCoeffs);
          } else if (nChannelNum == 1) {
            image =
                cv::Mat(stImageInfo.stFrameInfo.nHeight,
                        stImageInfo.stFrameInfo.nWidth, CV_8UC1, pConvertData);
          }

          cv::imshow("Camera Image", image);
          if (cv::waitKey(1) == 27) // 按下 ESC 键退出
          {
            break;
          }

          free(pConvertData);
          pConvertData = NULL;
        }
        MV_CC_FreeImageBuffer(handle, &stImageInfo);
      }
    }

    nRet = MV_CC_StopGrabbing(handle);
    if (MV_OK != nRet) {
      printf("Stop Grabbing fail! nRet [0x%x]\n", nRet);
      break;
    }

    nRet = MV_CC_CloseDevice(handle);
    if (MV_OK != nRet) {
      printf("Close Device fail! nRet [0x%x]\n", nRet);
      break;
    }

    nRet = MV_CC_DestroyHandle(handle);
    if (MV_OK != nRet) {
      printf("Destroy Handle fail! nRet [0x%x]\n", nRet);
      break;
    }
  } while (0);

  if (pConvertData) {
    free(pConvertData);
    pConvertData = NULL;
  }

  if (nRet != MV_OK) {
    if (handle != NULL) {
      MV_CC_DestroyHandle(handle);
      handle = NULL;
    }
  }

  printf("Press a key to exit.\n");
  WaitForKeyPress();
  return 0;
}
