/*******************************************************
 * Copyright (C) 2019
 *
 * Author: SongYang (ysong@aceinna.com)
 *******************************************************/
#pragma once
#include <opencv2/highgui/highgui.hpp>
#include "mynteye/api/api.h"
#include "driver.h"
#include "cv_painter.h"

MYNTEYE_USE_NAMESPACE

class CMyntS1030Driver : public CDriverBase
{
private:
    std::shared_ptr<API> m_api;
public:
    CMyntS1030Driver();
    virtual ~CMyntS1030Driver();

    virtual bool Init();
    virtual bool Init(int argc, char **argv);
    virtual void Start();
    virtual void Stop();
    virtual void ThreadGetData();

    SImgData ImageDataConvert(const api::StreamData &src);
    SImuData IMUDataConvert(const api::MotionData &src);
    SImgData ModifyImage(const cv::Mat &img, const SImgData &imgData);
};
