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
    Eigen::Vector3d ng1 = g.normalized();//SONG:g is {0,0,9.8}, need debug to check ng1 is {0,0,1}.
    Eigen::Vector3d ng2{0, 0, 1.0};
    R0 = Eigen::Quaterniond::FromTwoVectors(ng1, ng2).toRotationMatrix();
    /*
    SONG: 上式求得的旋转矩阵R0满足：R0*ng1 = ng2
    */

   //SONG: 又没有mag，此处的yaw怎么理解？需要debug看看。
    double yaw = Utility::R2ypr(R0).x(); //SONG: R2ypr由姿态矩阵R计算YPR.
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0; //SONG: 将roll和pitch强制置零。感觉这个地方是有问题的！！！比如初始位置是在斜坡上呢？？？
    // R0 = Utility::ypr2R(Eigen::Vector3d{-90, 0, 0}) * R0;
    return R0;
}
