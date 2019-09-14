/*******************************************************
 * Copyright (C) 2019
 *
 * Author: SongYang (ysong@aceinna.com)
 *******************************************************/
#pragma once
#include <fstream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>
#include "../estimator/estimator.h"
#include "../driver/macro.h"

class Estimator;

class CVisualBase
{
protected:

public:
    CVisualBase(){};
    virtual ~CVisualBase(){};

    virtual void Init() = 0;
    virtual void ShowTrackImage(cv::Mat track_img, double t) = 0;
    virtual void ShowLatestOdometry(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, const Eigen::Vector3d &V, double t) = 0;
    virtual void ShowResults(const Estimator &estimator, double timestamp) = 0;

};

