/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include <iostream>
using namespace std;

#include <ceres/ceres.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include "driver/mglog.h"

void yaml_read_write()
{
	std::string config_file = "/home/song/catkin_ws/src/VINS-Fusion_Learn/config/mynteye-s/mynt_mono_config.yaml";

	//read yaml file. 
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if (!fsSettings.isOpened()) {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

	std::string IMAGE0_TOPIC;
    fsSettings["image0_topic"] >> IMAGE0_TOPIC;
	cout << "image0_topic: " << IMAGE0_TOPIC << endl << endl;

    cv::Mat cv_T;
	fsSettings["body_T_cam0"] >> cv_T;
	Eigen::Matrix4d T;
	cv::cv2eigen(cv_T, T);
	cout << "body_T_cam0 Matrix: " << endl << T << endl << endl;

	//write yaml file. 
	cv::FileStorage fs("file.csv", cv::FileStorage::WRITE);

	// Eigen::Matrix4d eigen_T = Eigen::Matrix4d::Identity();
	cv::Mat cv_T_;
	cv::eigen2cv(T, cv_T_);
	fs << "body_T_cam_new" << cv_T_ ;
	fs.release();
}

// 正常来讲，一个四元数是不能和一个向量直接做乘法的，维度都对不上；
// 但Eigen::Quaterniond可以和向量v直接做乘法，其结果为q转为R后，R与向量v的乘积。
// 如下边的例子，v1等于v2。
void EigenTest1()
{
	Eigen::Quaterniond q(2, 0, 1, -3);
	q = q.normalized(); //验证了下，不做归一化不影响后边结果。
	cout << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << endl;

	Eigen::Matrix3d R = q.toRotationMatrix();
	Eigen::Vector3d v(0, 0, 9.8);

	Eigen::Vector3d v1 = q * v;
	cout << v1[0] << ", " << v1[1] << ", " << v1[2] << endl;

	Eigen::Vector3d v2 = R * v;
	cout << v2[0] << ", " << v2[1] << ", " << v2[2] << endl;
}

void Test1()
{
    queue<pair<double, Eigen::Vector3d>> accBuf;
	Eigen::Vector3d v(0, 0, 9.8);
    accBuf.push(make_pair(1, v));
    accBuf.push(make_pair(2, v));
    accBuf.push(make_pair(3, v));

	//验证此处是深拷贝还是浅拷贝。答案：深拷贝
    queue<pair<double, Eigen::Vector3d>> tmp_accBuf = accBuf;

    while(!tmp_accBuf.empty())
    {
        Eigen::Vector3d acc = tmp_accBuf.front().second;
        tmp_accBuf.pop();
		cout << accBuf.size() << endl;
    }
}

int main(int argc, char** argv)
{
	// cout << "Hello!" << endl;
	// yaml_read_write();

	// EigenTest1();

	Test1();
	return 0;
}
