/*******************************************************
 * Copyright (C) 2019
 *
 * Author: SongYang (ysong@aceinna.com)
 *******************************************************/
#include "visual_sdk.h"


CVisualSDK::CVisualSDK()
{
  // cout << "CVisualSDK:CVisualSDK()" << endl;
}

CVisualSDK::~CVisualSDK()
{
  // cout << "CVisualSDK:~CVisualSDK()" << endl;
}

void CVisualSDK::Init()
{

}

void CVisualSDK::ShowTrackImage(cv::Mat track_img, double t)
{
    //show in opencv window !!!!!
}

void CVisualSDK::ShowLatestOdometry(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, const Eigen::Vector3d &V, double t)
{

}

void CVisualSDK::ShowResults(const Estimator &estimator, double timestamp)
{
  
}



