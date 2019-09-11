/*******************************************************
 * Copyright (C) 2019
 *
 * Author: SongYang (ysong@aceinna.com)
 *******************************************************/
#pragma once
#include <iostream>
#include <unistd.h>
#include <thread>
#include <mutex>
#include <opencv2/opencv.hpp>
#include "struct.h"
#include <glog/logging.h>
#include "macro.h"
#include "../estimator/estimator.h"
#include "simulator.h"


using namespace std;

class CAdapter 
{
private:
    Estimator estimator;
    queue<SImuData> imu_buf;
    // queue<sensor_msgs::PointCloudConstPtr> feature_buf;
    queue<SImgData> img0_buf;
    queue<SImgData> img1_buf;
    std::mutex m_buf;
    CSimulator simulator;
    std::thread thread_sync;

public:
    CAdapter();
    ~CAdapter();

    void UpdateLeftImage (SImgData& img);
    void UpdateRightImage(SImgData& img);
    void UpdateIMU(SImuData& imu);
private:
    void sync_process();
};