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

#pragma once

#include <cstdio>
#include <iostream>
#include <queue>
#include <execinfo.h>
#include <csignal>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include "../estimator/parameters.h"
#include "../utility/tic_toc.h"

using namespace std;
using namespace camodocal;
using namespace Eigen;

bool inBorder(const cv::Point2f &pt);
void reduceVector(vector<cv::Point2f> &v, vector<uchar> status);
void reduceVector(vector<int> &v, vector<uchar> status);

class FeatureTracker
{
public:
    FeatureTracker();
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> trackImage(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1 = cv::Mat());
    void setMask();
    void readIntrinsicParameter(const vector<string> &calib_file);
    void showUndistortion(const string &name);
    void rejectWithF();
    void undistortedPoints();
    vector<cv::Point2f> undistortedPts(vector<cv::Point2f> &pts, camodocal::CameraPtr cam);
    vector<cv::Point2f> ptsVelocity(vector<int> &ids, vector<cv::Point2f> &pts, 
                                    map<int, cv::Point2f> &cur_id_pts, map<int, cv::Point2f> &prev_id_pts);
    void showTwoImage(const cv::Mat &img1, const cv::Mat &img2, 
                      vector<cv::Point2f> pts1, vector<cv::Point2f> pts2);
    void drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight, 
                                   vector<int> &curLeftIds,
                                   vector<cv::Point2f> &curLeftPts, 
                                   vector<cv::Point2f> &curRightPts,
                                   map<int, cv::Point2f> &prevLeftPtsMap);
    void setPrediction(map<int, Eigen::Vector3d> &predictPts);
    double distance(cv::Point2f &pt1, cv::Point2f &pt2);
    void removeOutliers(set<int> &removePtsIds);
    cv::Mat getTrackImage();
    bool inBorder(const cv::Point2f &pt);

    // 下边这些变量:
    // 带"_un_"自带的表示非畸变的特征点。
    // cur: 当前帧， prev:上一帧
    // right: 当使用双目相机时，用于标识右目相机. （单目相机对应左目）
    // 关于成员变量的组织(设计)不是很好，可以将特征点，id，时间戳，跟踪数量等信息封装成一个struct。
    int row, col; //图像的行和列值
    cv::Mat imTrack;
    cv::Mat mask;
    cv::Mat fisheye_mask;
    cv::Mat prev_img, cur_img; //上一帧和当前帧的原始畸变图像。
    vector<cv::Point2f> n_pts; //保存由goodFeaturesToTrack得到的角点。
    vector<cv::Point2f> predict_pts; //预测的特征点
    vector<cv::Point2f> predict_pts_debug;
    vector<cv::Point2f> prev_pts, cur_pts, cur_right_pts; //上一帧、当前帧的特征点容器。
    vector<cv::Point2f> prev_un_pts, cur_un_pts, cur_un_right_pts;//上一帧、当前帧的非畸变特征点容器。
    vector<cv::Point2f> pts_velocity, right_pts_velocity; //用于存储相邻两帧图像得到的对应特征点的相对速度。
    vector<int> ids, ids_right; //每个特征点都有一个id, ids就是特征点的id容器，次序和cur_pts一一对应。
    vector<int> track_cnt; //记录了每个特征点被连续跟踪了多少帧，次序和cur_pts一一对应，即特征点cur_pts[i]被连续跟踪了track_cnt[i]帧。
    map<int, cv::Point2f> cur_un_pts_map, prev_un_pts_map; //[id, 畸变矫正后的特征点坐标]
    map<int, cv::Point2f> cur_un_right_pts_map, prev_un_right_pts_map;
    map<int, cv::Point2f> prevLeftPtsMap;//[id, 未做畸变矫正的特征点坐标]
    vector<camodocal::CameraPtr> m_camera; //用于pinhole、mei和kannala_brandt等相机模型的相关处理，比如特征点畸变矫正，将特征点投影到归一化平面。
    double cur_time; //当前帧的时间戳
    double prev_time;//上一帧的时间戳
    bool stereo_cam; //是否为双目相机的标志位，默认为false，即单目相机。
    int n_id;  //id生成器，用于生成新的id号，用n_id++作为新特征点的id。
    bool hasPrediction;
};
