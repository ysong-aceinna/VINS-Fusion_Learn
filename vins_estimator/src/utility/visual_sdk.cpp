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
    cv::imshow("Track Image", track_img);
    cv::waitKey(1);
}

void CVisualSDK::ShowLatestOdometry(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, const Eigen::Vector3d &V, double t)
{

}

void CVisualSDK::ShowResults(const Estimator &estimator, double timestamp)
{
    ShowPositionAttitude(estimator, timestamp);
}

void CVisualSDK::ShowPositionAttitude(const Estimator &estimator, double timestamp)
{
    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
    {
        Quaterniond tmp_Q;
        tmp_Q = Quaterniond(estimator.Rs[WINDOW_SIZE]);
        Eigen::Vector3d YPR = Eigen::Quaterniond(tmp_Q.w(), tmp_Q.x(), tmp_Q.y(), tmp_Q.z()).matrix().eulerAngles(2,1,0)/ M_PI * 180.0;
        // cout << timestamp 
        //         << estimator.Ps[WINDOW_SIZE].x() << ","
        //         << estimator.Ps[WINDOW_SIZE].y() << ","
        //         << estimator.Ps[WINDOW_SIZE].z() << ","
        //         // << tmp_Q.w() << ","
        //         // << tmp_Q.x() << ","
        //         // << tmp_Q.y() << ","
        //         // << tmp_Q.z() << ","
        //         << YPR.x() << ","
        //         << YPR.y() << ","
        //         << YPR.z() << ","
        //         << estimator.Vs[WINDOW_SIZE].x() << ","
        //         << estimator.Vs[WINDOW_SIZE].y() << ","
        //         << estimator.Vs[WINDOW_SIZE].z() << "," << endl;
        Eigen::Vector3d tmp_T = estimator.Ps[WINDOW_SIZE];
        printf("time: %f, t: %f %f %f q: %f %f %f %f YPR: %f %f %f \n", timestamp, tmp_T.x(), tmp_T.y(), tmp_T.z(),
                                                          tmp_Q.w(), tmp_Q.x(), tmp_Q.y(), tmp_Q.z(),
                                                          YPR.x(), YPR.y(), YPR.z());

    }
}