/*******************************************************
 * Copyright (C) 2019
 *
 * Author: SongYang (ysong@aceinna.com)
 *******************************************************/
#pragma once
#include "visual_base.h"

class CVisualSDK : public CVisualBase
{
private:
public:
    CVisualSDK();
    virtual ~CVisualSDK();
    virtual void Init();
    virtual void ShowTrackImage(cv::Mat track_img, double t);
    virtual void ShowLatestOdometry(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, const Eigen::Vector3d &V, double t);
    virtual void ShowResults(const Estimator &estimator, double timestamp);

    void ShowPositionAttitude(const Estimator &estimator, double timestamp);

};
