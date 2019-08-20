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


int main(int argc, char** argv)
{
	cout << "Hello!" << endl;
	yaml_read_write();
	return 0;
}
