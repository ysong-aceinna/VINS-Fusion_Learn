/*******************************************************
 * Copyright (C) 2019
 *
 * Author: SongYang (ysong@aceinna.com)
 *******************************************************/
#include "driver.h"

CDriverBase::CDriverBase()
{ 
    cout << "CDriverBase:CDriverBase()" << endl;
    m_padapter = NULL;
    m_img_fps = 0;
    m_imu_odr = 0;
    m_width = 0;
    m_height = 0;
    m_bROS = false;
    m_bexit = false;
    m_bmono = false;
    m_image0_topic = "";
    m_image1_topic = "";
    m_imu_topic = "";
    m_use_imu = 0;
}

CDriverBase::~CDriverBase()
{
    LOG(INFO) << "CDriverBase:~CDriverBase()";
    SAFEDELETE(m_padapter);
}

void CDriverBase::AddListener(CAdapter* p)
{
    m_padapter = p;
}

void CDriverBase::ReadParameters(std::string config_file)
{
    // FILE *fh = fopen(config_file.c_str(),"r");
    // if(fh == NULL){
    //     LOG(ERROR) << "config_file dosen't exist; wrong config_file path";
    //     return;          
    // }
    // fclose(fh);

    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        LOG(ERROR) << "ERROR: Wrong path to settings.";
    }

    fsSettings["image0_topic"] >> m_image0_topic;
    fsSettings["image1_topic"] >> m_image1_topic;
    m_use_imu = fsSettings["imu"];
    if(m_use_imu)
    {
        fsSettings["imu_topic"] >> m_imu_topic;
    }

    int num_of_cam = fsSettings["num_of_cam"];
    if(num_of_cam == 1)
    {
        m_bmono = true;
    }
    else if(num_of_cam == 2)
    {
        m_bmono = false;
    }

    m_width = fsSettings["image_width"];
    m_height = fsSettings["image_height"];
    m_img_fps = fsSettings["image_fps"];
    m_imu_odr = fsSettings["imu_odr"];

    fsSettings.release();
}
