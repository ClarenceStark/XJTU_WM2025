#ifndef _LIB_VYAW
#define _LIB_VYAW
#include "opencv2/core/types.hpp"
#include <chrono>

#include <cstdint>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <queue>
#include <random>
#include <stdlib.h>
#include <time.h>
#include <vector>

#include <globalParam.hpp>
#include <tracker.hpp>

class calculate_
{
private:
    uint8_t current_state;
    uint8_t last_state;

    double yaw, last_yaw;
    double x, last_x;
    uint32_t count = 0;
    std::vector<double> yaw_list;
    std::vector<double> x_list;
    // std::vector<int> state_list;
    double start_time = 0, last_time = 0, end_time = 0;
    double vyaw, vx;

    bool time_state = 0;

public:
    // void update_in_tracker(Tracker &tc, double time)
    // {
    //     //std::cout << tc.tracker_state;
    //     last_state = current_state;
    //     current_state = tc.tracker_state;
    //     if (current_state == Tracker::LOST or current_state == Tracker::TEMP_LOST)
    //     {
    //         if (time_state == 0)
    //         {
    //             end_time = last_time;
    //             time_state = 1;
    //         }
    //         return;
    //     }

    //     // start_time = time;

    //     yaw = tc.target_state(1);
    //     //std::cout << "                          tagtag" << yaw << "\n";
    //     bool switch_cond = (abs(yaw - last_yaw) > 0.01 && !(yaw_list.empty()));
    //     if (switch_cond)
    //     {
    //         // jump,进行结算
    //         // //std::cout << "delta yaw::::::::::::::" << yaw_list.front() - yaw_list.back() << "   " << abs(yaw - last_yaw) << std::endl;
    //         if (yaw_list.size() > 2)
    //         {
    //             vyaw = (yaw_list.front() - yaw_list.back()) / (end_time - start_time);
    //             //std::cout << "vyaw=========" << vyaw << " " << (end_time - start_time) << " " << count << std::endl;
    //         }

    //         for (auto i : yaw_list)
    //         {
    //             //std::cout << i << std::endl;
    //         }
    //         yaw_list.clear();
    //         count = 0;
    //         start_time = time;
    //         time_state = 0;
    //     }

    //     if (current_state == Tracker::TRACKING or current_state == Tracker::DETECTING)
    //     {
    //         yaw_list.push_back(yaw);
    //     }
    //     count++;
    //     last_yaw = yaw;
    //     end_time = last_time = time;
    // }

    void update_in_tarlist(std::deque<Armor> &tar_list, double time)
    {
        last_state = current_state;
        current_state = tar_list.size();

        if (tar_list.size() == 0)
        {
            cv::waitKey(10000);
            if (time_state == 0)
            {
                end_time = last_time;
                time_state = 1;
            }
            return;
        }

        x = tar_list.front().position(1);

        ////std::cout << "                          tagtag" << x << "\n";

        bool switch_cond = (abs(x - last_x) > 0.2 && !(x_list.empty()));
        if (switch_cond)
        {
            if (x_list.size() > 2)
            {
                vx = (x_list.front() - x_list.back()) / (end_time - start_time);
                ////std::cout << "vx=========" << vx * 1000 << " " << (end_time - start_time) << " " << count << std::endl;
                int a = 1;
            }

            for (auto i : x_list)
            {
                ////std::cout << i << std::endl;
            }
            x_list.clear();
            count = 0;
            start_time = time;
            time_state = 0;
        }

        if (tar_list.size() != 0)
        {
            x_list.push_back(x);
        }
        count++;
        last_x = x;
        end_time = last_time = time;
    }
};

#endif // _LIB_VYAW