/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "pose_local_parameterization.h"
//实现 f(x + delta)
bool PoseLocalParameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const
{
    //Map类用于通过C++中普通的连续指针或者数组 （raw C/C++ arrays）来构造Eigen里的Matrix类，
    //这就好比Eigen里的Matrix类的数据和raw C++array 共享了一片地址，也就是引用。
    Eigen::Map<const Eigen::Vector3d> _p(x);
    Eigen::Map<const Eigen::Quaterniond> _q(x + 3);

    Eigen::Map<const Eigen::Vector3d> dp(delta);

    Eigen::Quaterniond dq = Utility::deltaQ(Eigen::Map<const Eigen::Vector3d>(delta + 3));

    Eigen::Map<Eigen::Vector3d> p(x_plus_delta);
    Eigen::Map<Eigen::Quaterniond> q(x_plus_delta + 3);

    p = _p + dp; //对于pose，pose = _pose + delta_pose
    q = (_q * dq).normalized();  //对于姿态，q = _q * delta_q, 然后归一化
    //修改p q, 即修改x_plus_delta.
    return true;
}
bool PoseLocalParameterization::ComputeJacobian(const double *x, double *jacobian) const
{
    Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
    j.topRows<6>().setIdentity();
    j.bottomRows<1>().setZero();

    /*暂且这么理解，不确定正确性：
    1  0  0  0  0  0    : pos_x
    0  1  0  0  0  0    : pos_y
    0  0  1  0  0  0    : pos_z
    0  0  0  1  0  0    : q_x
    0  0  0  0  1  0    : q_y
    0  0  0  0  0  1    : q_z
    0  0  0  0  0  0    : q_s 对四元数而言，多了一个自由度(标量部分)，所以求偏导都是0
    */
    return true;
}
