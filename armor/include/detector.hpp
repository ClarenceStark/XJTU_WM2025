// Copyright 2022 Chen Jun
// Licensed under the MIT License.

#ifndef ARMOR_DETECTOR__DETECTOR_HPP_
#define ARMOR_DETECTOR__DETECTOR_HPP_

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>

// STD
#include <cmath>
#include <string>
#include <vector>

#include "armor.hpp"
#include "number_classifier.hpp"
#include "globalParam.hpp"

class Detector{    
    public:
        struct LightParams
        {
            // width / height
            double min_ratio;
            double max_ratio;
            // vertical angle
            double max_angle;
        };

        struct ArmorParams
        {
            double min_light_ratio;
            // light pairs distance
            double min_small_center_distance;
            double max_small_center_distance;
            double min_large_center_distance;
            double max_large_center_distance;
            // horizontal angle
            double max_angle;
        };

        Detector(GlobalParam &gp);

        std::vector<UnsolvedArmor> detect(const cv::Mat &input, const int color);

        cv::Mat preprocessImage(const cv::Mat &input);
        std::vector<Light> findLights(const cv::Mat &rbg_img, const cv::Mat &binary_img);
        std::vector<UnsolvedArmor> matchLights(const std::vector<Light> &lights);

        // For debug usage
        cv::Mat getAllNumbersImage();
        void drawResults(cv::Mat &img);

        // parameters
        int blue_threshold, red_threshold;
        int binary_thres;
        int detect_color;
        LightParams l;
        ArmorParams a;
        std::unique_ptr<NumberClassifier> classifier;

        // Debug msgs
        cv::Mat binary_img;

    private:
        bool isLight(const Light &possible_light);
        bool containLight(
            const Light &light_1, const Light &light_2, const std::vector<Light> &lights);
        ArmorType isArmor(const Light &light_1, const Light &light_2);

        std::vector<Light> lights_;
        std::vector<UnsolvedArmor> armors_;
};

#endif // ARMOR_DETECTOR__DETECTOR_HPP_
