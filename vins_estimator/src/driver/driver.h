/*******************************************************
 * Copyright (C) 2019
 *
 * Author: SongYang (ysong@aceinna.com)
 *******************************************************/
#pragma once

#include <thread>
#include <mutex>
#include "adapter.h"
#include "struct.h"

class CDriverBase
{
protected:
    int m_fps;        // frames per second of images
    int m_odr;        // odr of imu
    int m_width;      // width of image
    int m_height;     // height of image
    bool m_bROS;
    CAdapter* m_padapter;
    bool m_bexit;
    std::mutex m_mx_exit;
    std::thread m_get_data_thread;

    // ROS
    std::string m_image0_topic, m_image1_topic, m_imu_topic;
    int m_use_imu;
    bool m_bmono;

    const double R2D = 180.0/M_PI;
    const double G_N = 9.8; //g_normal

public:
    CDriverBase();
    virtual ~CDriverBase();

    // void SetParameters(int fps, int odr, int width, int height, bool use_imu, bool b_mono)
    // {
    //     m_fps = fps;
    //     m_odr = odr;
    //     m_width = width;
    //     m_height = height;        
    //     m_use_imu = use_imu;
    //     m_bmono = b_mono;
    // }

    virtual int Init() = 0;
    virtual int Init(int argc, char **argv) = 0;
    virtual void Start() = 0;
    virtual void Stop() = 0;
    virtual void ThreadGetData() = 0;

    void AddListener(CAdapter* p);
    void ReadParameters(std::string config_file);
};

