#include "tracker.hpp"
#include "globalParam.hpp"

// STD

// static inline double normalize_angle(double angle)
// {
//     const double result = fmod(angle + M_PI, 2.0 * M_PI);
//     if (result <= 0.0)
//         return result + M_PI;
//     return result - M_PI;
// }
// double shortest_angular_distance(double from, double to)
// {
//     return normalize_angle(to - from);
// }

// x << xc, vx, yc, vy, z1, z2, vz, r1, r2, yaw, vyaw;

inline double dyaw(double yaw1, double yaw2){
    double ans = fmod(abs(yaw1 - yaw2), 2*M_PI);
    return  min(ans, 2*M_PI - ans);
}

inline double cost(Armor &a, Armor &b){
    double ans = 0;
    cv::Point3f p1 = a.center;
    cv::Point3f p2 = b.center;
    ans = sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2));
    ans += 50 * pow(dyaw(a.yaw, b.yaw), 2);
    return ans;
}

inline Armor calcArmor(double xc, double yc, double z, double r, double yaw){
    Armor armor;
    armor.center = cv::Point3f(xc - r * cos(yaw), yc - r * sin(yaw), z);
    armor.yaw = yaw;
    return armor;
}

void Tracker::track(std::vector<Armor> &armors_curr, Translator &ts, double dt){
    this->dt = dt;
    for (auto &zVector : z_vector_list) zVector.setZero();
    armors_pred.clear();
    for (auto &ekf : ekf_list){
        auto x = ekf.predict();
        armors_pred.push_back(calcArmor(x(0), x(2), x(4), x(7), x(9)));
        armors_pred.push_back(calcArmor(x(0), x(2), x(5), x(8), x(9) + M_PI/2));
        armors_pred.push_back(calcArmor(x(0), x(2), x(4), x(7), x(9) + M_PI));
        armors_pred.push_back(calcArmor(x(0), x(2), x(5), x(8), x(9) + 3*M_PI/2));
    };
    std::vector<int> matchX, matchY;
    int n, m;
    while(true){
        bool all_matched = true;
        n = armors_curr.size();
        m = armors_pred.size();
        KuhnMunkres km;
        std::vector<std::vector<double>> w(n, std::vector<double>(m, -INF));
        for(int i = 0; i < n; i++){
            for(int j = 0; j < m; j++){
                auto &u = armors_curr[i];
                auto &v = armors_pred[j];
                w[i][j] = cost(u, v);
            }       
        }
        km.solve(w, matchX, matchY, gp->cost_threshold);
        for(int i = 0; i < n; i++)
            if(matchX[i] == -1){
                all_matched = false;
                create_new_ekf(armors_curr[i]);
                break;
            }
        if(all_matched) break;
    }
    for(int i = 0; i < n; i++){
        int EkfID   = matchX[i]/4;
        int ArmorID = matchX[i]%4;
        auto &armor = armors_curr[i];
        z_vector_list[EkfID].segment(ArmorID*4, 4) << armor.center.x, armor.center.y, armor.center.z, armor.yaw;
    }
    for (int i = 0; i < ekf_list.size(); i++){
        if (z_vector_list[i].norm() == 0){
            lost_frame_count[i]++;
            if (lost_frame_count[i] > gp->max_lost_frame){
                ekf_list.erase(ekf_list.begin() + i);
                z_vector_list.erase(z_vector_list.begin() + i);
                lost_frame_count.erase(lost_frame_count.begin() + i);
                i--;
            }
            continue;
        }
        refine_zVector(i);
        ekf_list[i].update(z_vector_list[i]);
    }
    if (ekf_list.size() > 0){
        // ts = ekf_list[0].get_X();
    }
}

void Tracker::refine_zVector(int ekf_id){
    auto x = ekf_list[ekf_id].get_X();
    double xc = x(0), yc = x(2), z1 = x(4), z2 = x(5), r1 = x(7), r2 = x(8);
    auto &z = z_vector_list[ekf_id];
    // 看不见的装甲板的位姿由能看见的装甲板估计
    if (z.segment(0, 4) != Eigen::VectorXd::Zero(4)){
        auto armor = calcArmor(xc, yc, z(2), r1, z(3) + M_PI);
        z.segment(8, 4) << armor.center.x, armor.center.y, armor.center.z, armor.yaw;
    }else if (z.segment(8, 4) != Eigen::VectorXd::Zero(4)){
        auto armor = calcArmor(xc, yc, z(10), r1, z(11) - M_PI);
        z.segment(0, 4) << armor.center.x, armor.center.y, armor.center.z, armor.yaw;
    }
    if (z.segment(4, 4) != Eigen::VectorXd::Zero(4)){
        auto armor = calcArmor(xc, yc, z(6), r2, z(7) + M_PI);
        z.segment(12, 4) << armor.center.x, armor.center.y, armor.center.z, armor.yaw;
    }else if (z.segment(12, 4) != Eigen::VectorXd::Zero(4)){
        auto armor = calcArmor(xc, yc, z(14), r2, z(15) - M_PI);
        z.segment(4, 4) << armor.center.x, armor.center.y, armor.center.z, armor.yaw;
    }
    if (z.segment(0, 4) == Eigen::VectorXd::Zero(4) && z.segment(8, 4) == Eigen::VectorXd::Zero(4)){
        auto armor = calcArmor(xc, yc, z1, r1, z(7) - M_PI/2);
        z.segment(0, 4) << armor.center.x, armor.center.y, armor.center.z, armor.yaw;
        auto armor = calcArmor(xc, yc, z1, r1, z(7) + M_PI/2);
        z.segment(8, 4) << armor.center.x, armor.center.y, armor.center.z, armor.yaw;
    }
    if (z.segment(4, 4) == Eigen::VectorXd::Zero(4) && z.segment(12, 4) == Eigen::VectorXd::Zero(4)){
        auto armor = calcArmor(xc, yc, z2, r2, z(3) + M_PI/2);
        z.segment(4, 4) << armor.center.x, armor.center.y, armor.center.z, armor.yaw;
        auto armor = calcArmor(xc, yc, z2, r2, z(3) - M_PI/2);
        z.segment(12, 4) << armor.center.x, armor.center.y, armor.center.z, armor.yaw;
    }
}

void Tracker::create_new_ekf(Armor &armor){
    Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(11, 11) * gp->s2p0;
    cv::Point3f c = calcArmor(armor.center.x, armor.center.y, armor.center.z, gp->r_initial, armor.yaw + M_PI).center; 
    Eigen::VectorXd x0;
    x0 << c.x, 0, c.y, 0, c.z, c.z, 0, gp->r_initial, gp->r_initial, armor.yaw, 0;
    ekf_list.push_back(ExtendedKalmanFilter(f, h, j_f, j_h, u_q, u_r, nomolize_residual, P0, x0));
    z_vector_list.push_back(Eigen::VectorXd::Zero(16));
    lost_frame_count.push_back(0);
    armors_pred.push_back(calcArmor(x0(0), x0(2), x0(4), x0(7), x0(9)));
    armors_pred.push_back(calcArmor(x0(0), x0(2), x0(5), x0(8), x0(9) + M_PI/2));
    armors_pred.push_back(calcArmor(x0(0), x0(2), x0(4), x0(7), x0(9) + M_PI));
    armors_pred.push_back(calcArmor(x0(0), x0(2), x0(5), x0(8), x0(9) + 3*M_PI/2));
}

Tracker::Tracker(GlobalParam &gp){
    this->gp = &gp;

    // 定义状态转移函数
    f = [this](const Eigen::VectorXd &x)
    {
        // 更新状态：位置和速度
        Eigen::VectorXd x_new = x;
        x_new(0) += x(1) * dt; // 更新xc
        x_new(2) += x(3) * dt; // 更新yc
        x_new(4) += x(6) * dt; // 更新z1
        x_new(5) += x(6) * dt; // 更新z2
        x_new(9) += x(10)* dt; // 更新yaw
        return x_new;
    };

    // 状态转移函数的雅可比矩阵
    j_f = [this](const Eigen::VectorXd &)
    {
        Eigen::MatrixXd f(9, 9);
        // clang-format off
        //    xc   vx   yc   vy   z1   z2   vz   r1   r2   yaw  vyaw
        f <<  1,   dt,  0,   0,   0,   0,   0,   0,   0,   0,   0,
              0,   1,   0,   0,   0,   0,   0,   0,   0,   0,   0,
              0,   0,   1,   dt,  0,   0,   0,   0,   0,   0,   0,
              0,   0,   0,   1,   0,   0,   0,   0,   0,   0,   0,
              0,   0,   0,   0,   1,   0,   dt,  0,   0,   0,   0,
              0,   0,   0,   0,   0,   1,   dt,  0,   0,   0,   0,
              0,   0,   0,   0,   0,   0,   1,   0,   0,   0,   0,
              0,   0,   0,   0,   0,   0,   0,   1,   0,   0,   0,
              0,   0,   0,   0,   0,   0,   0,   0,   1,   0,   0,
              0,   0,   0,   0,   0,   0,   0,   0,   0,   1,   dt,
              0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   1;
        // clang-format on
        return f;
    };

    // 观测函数
    h = [](const Eigen::VectorXd &x)
    {
        Eigen::VectorXd z(4);
        double xc = x(0), yc = x(2), yaw = x(9), z1 = x(4), z2 = x(5), r1 = x(7), r2 = x(8);
        z(0) = xc - r1 * cos(yaw); 
        z(1) = yc - r1 * sin(yaw); 
        z(2) = z1;              
        z(3) = yaw;              

        yaw += M_PI/2;
        z(4) = xc - r2 * cos(yaw); 
        z(5) = yc - r2 * sin(yaw); 
        z(6) = z2;              
        z(7) = yaw;              

        yaw += M_PI/2;
        z(8) = xc - r1 * cos(yaw); 
        z(9) = yc - r1 * sin(yaw); 
        z(10)= z1;              
        z(11)= yaw;              

        yaw += M_PI/2;
        z(12)= xc - r2 * cos(yaw); 
        z(13)= yc - r2 * sin(yaw); 
        z(14)= z2;              
        z(15)= yaw;              
        return z;
    };

    // 观测函数的雅可比矩阵
    j_h = [](const Eigen::VectorXd &x)
    {
        Eigen::MatrixXd h(4, 9);
        double yaw = x(6), r1 = x(7), r2 = x(8);
        // clang-format off
        //    xc   vx   yc   vy   z1   z2   vz   r1              r2                  yaw                vyaw
        h <<  1,   0,   0,   0,   0,   0,   0,   -cos(yaw),      0,                  r1*sin(yaw),           0,
              0,   0,   1,   0,   0,   0,   0,   -sin(yaw),      0,                  -r1*cos(yaw),          0,
              0,   0,   0,   0,   1,   0,   0,   0,              0,                  0,                     0,
              0,   0,   0,   0,   0,   0,   0,   0,              0,                  1,                     0,

              1,   0,   0,   0,   0,   0,   0,   0,              -cos(yaw+M_PI/2),   r2*sin(yaw+M_PI/2),    0,
              0,   0,   1,   0,   0,   0,   0,   0,              -sin(yaw+M_PI/2),   -r2*cos(yaw+M_PI/2),   0,
              0,   0,   0,   0,   0,   1,   0,   0,              0,                  0,                     0,
              0,   0,   0,   0,   0,   0,   0,   0,              0,                  0,                     0,
              0,   0,   0,   0,   0,   0,   0,   0,              0,                  1,                     0,

              1,   0,   0,   0,   0,   0,   0,   -cos(yaw+M_PI), 0,                  r1*sin(yaw+M_PI),      0,
              0,   0,   1,   0,   0,   0,   0,   -sin(yaw+M_PI), 0,                  -r1*cos(yaw+M_PI),     0,
              0,   0,   0,   0,   1,   0,   0,   0,              0,                  0,                     0,
              0,   0,   0,   0,   0,   0,   0,   0,              0,                  1,                     0,
              
              1,   0,   0,   0,   0,   0,   0,   0,              -cos(yaw+3*M_PI/2), r2*sin(yaw+3*M_PI/2),  0,
              0,   0,   1,   0,   0,   0,   0,   0,              -sin(yaw+3*M_PI/2), -r2*cos(yaw+3*M_PI/2), 0,                 
              0,   0,   0,   0,   0,   1,   0,   0,              0,                  0,                     0,
              0,   0,   0,   0,   0,   0,   0,   0,              0,                  1,                     0;
              
        // clang-format on
        return h;
    };

    // 过程噪声协方差矩阵 u_q_0
    u_q = [this, &gp]()
    {
        Eigen::MatrixXd q(9, 9);
        double t{dt}, x{gp.s2qxyz}, y{gp.s2qyaw}, r{gp.s2qr};
        // 计算各种噪声参数
        double q_x_x{pow(t, 4) / 4 * x}, q_x_vx{pow(t, 3) / 2 * x}, q_vx_vx{pow(t, 2) * x};
        double q_y_y{pow(t, 4) / 4 * y}, q_y_vy{pow(t, 3) / 2 * y}, q_vy_vy{pow(t, 2) * y};
        double q_r{pow(t, 4) / 4 * r};
        //    xc      vx      yc      vy      z1      z2      vz      r1      r2      yaw     vyaw
        q <<  q_x_x,  q_x_vx, 0,      0,      0,      0,      0,      0,      0,      0,      0,
              q_x_vx, q_vx_vx,0,      0,      0,      0,      0,      0,      0,      0,      0,
              0,      0,      q_x_x,  q_x_vx, 0,      0,      0,      0,      0,      0,      0,
              0,      0,      q_x_vx, q_vx_vx,0,      0,      0,      0,      0,      0,      0,
              0,      0,      0,      0,      q_x_x,  0,      q_x_vx, 0,      0,      0,      0,
              0,      0,      0,      0,      0,      q_vx_vx,q_x_vx, 0,      0,      0,      0,
              0,      0,      0,      0,      q_x_vx, q_x_vx, q_vx_vx,0,      0,      0,      0,
              0,      0,      0,      0,      0,      0,      0,      q_r,    0,      0,      0,
              0,      0,      0,      0,      0,      0,      0,      0,      q_r,    0,      0,
              0,      0,      0,      0,      0,      0,      0,      0,      0,      q_y_y,  q_y_vy,
              0,      0,      0,      0,      0,      0,      0,      0,      0,      q_y_vy, q_vy_vy;
        // clang-format on
        return q;
    };

    // 观测噪声协方差矩阵 u_r_0
    u_r = [this, &gp](const Eigen::VectorXd &z)
    {
        Eigen::DiagonalMatrix<double, 16> r;
        double xy = gp.r_xy_factor;
        r.diagonal() << abs(xy * z[0]),  abs(xy * z[1]),  gp.r_z, gp.r_yaw,
                        abs(xy * z[4]),  abs(xy * z[5]),  gp.r_z, gp.r_yaw,
                        abs(xy * z[8]),  abs(xy * z[9]),  gp.r_z, gp.r_yaw,
                        abs(xy * z[12]), abs(xy * z[13]), gp.r_z, gp.r_yaw; // 定义观测噪声
        return r;
    };

    nomolize_residual = [](const Eigen::VectorXd &z)
    {
        Eigen::VectorXd nz = z;
        nz(3) = atan2(sin(nz(3)), cos(nz(3)));
        nz(7) = atan2(sin(nz(7)), cos(nz(7)));
        nz(11) = atan2(sin(nz(11)), cos(nz(11)));
        nz(15) = atan2(sin(nz(15)), cos(nz(15)));
        return nz;
    };
}