/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "utility.h"

/*
SONG: 根据输入的向量g，求姿态矩阵。
*/
Eigen::Matrix3d Utility::g2R(const Eigen::Vector3d &g)
{
    Eigen::Matrix3d R0;
    /*
    SONG:debug_info:
                g: -0.579822 -0.541499 9.757808  
                ng1: -0.059226 -0.055311 0.996711 
    */
    Eigen::Vector3d ng1 = g.normalized();
    Eigen::Vector3d ng2{0, 0, 1.0};
    R0 = Eigen::Quaterniond::FromTwoVectors(ng1, ng2).toRotationMatrix(); //SONG: 旋转矩阵R0满足：R0*ng1 = ng2

    double yaw = Utility::R2ypr(R0).x(); //SONG: R2ypr由姿态矩阵R计算YPR. debug_info: yaw=0;
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    /*
    SONG:debug_info:Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}):
        1 0 0
        0 1 0
        0 0 1
    */

    // R0 = Utility::ypr2R(Eigen::Vector3d{-90, 0, 0}) * R0;
    return R0;
}
