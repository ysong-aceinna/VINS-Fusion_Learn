/*******************************************************
 * Copyright (C) 2019
 *
 * Author: SongYang (ysong@aceinna.com)
 *******************************************************/
#include "mynt_S1030.h"


CMyntS1030Driver::CMyntS1030Driver()
{
  cout << "CMyntS1030Driver:CMyntS1030Driver()" << endl;
  m_fps = 0;
  m_odr = 0;
  m_width = 752;
  m_height = 480;
}

CMyntS1030Driver::~CMyntS1030Driver()
{
  cout << "CMyntS1030Driver:~CMyntS1030Driver()" << endl;
}

int CMyntS1030Driver::Init()
{
  return 0;
}

int CMyntS1030Driver::Init(int argc, char **argv)
{
  m_api = API::Create(argc, argv); //"/mnt/wind/project/camera-driver/test/build/driver_demo"
  if (!m_api)
    return 1;
  auto request = m_api->GetStreamRequest();
  request.width = m_width;
  request.height = m_height;
  m_api->ConfigStreamRequest(request);

  return 0;
}

void CMyntS1030Driver::Start()
{
  LOG(INFO) << "CMyntS1030Driver::Start()";
  // Enable this will cache the motion datas until you get them.
  m_api->EnableMotionDatas();
  m_api->Start(Source::ALL);
  m_mx_exit.lock();
  m_bexit = false;
  m_mx_exit.unlock();
  m_get_data_thread = std::thread(&CMyntS1030Driver::ThreadGetData, this);
    // std::chrono::milliseconds dura(2010000000);
    // std::this_thread::sleep_for(dura);
}

void CMyntS1030Driver::Stop()
{
  // sleep(0.01); //sleep 10ms to wait for ThreadGetData finish work.
  m_mx_exit.lock();
  m_bexit = true;
  m_mx_exit.unlock();
  m_get_data_thread.join();
  m_api->Stop(Source::ALL);
  LOG(INFO) << "CMyntS1030Driver::Stop()";
}

void CMyntS1030Driver::ThreadGetData()
{
  LOG(INFO) << "CMyntS1030Driver::ThreadGetData()";
  CVPainter painter;

  while (1)
  {
    m_mx_exit.lock();
    if(m_bexit) 
    {
      m_mx_exit.unlock();
      break;
    }
    m_mx_exit.unlock();

    m_api->WaitForStreams();

    auto &&left_data = m_api->GetStreamData(Stream::LEFT);
    auto &&right_data = m_api->GetStreamData(Stream::RIGHT);
    auto &&motion_datas = m_api->GetMotionDatas();

    SImgData img_data;
    if (!left_data.frame.empty()) 
    {
      img_data = ImageDataConvert(left_data);
      m_padapter->UpdateLeftImage(img_data);
    }

    if (!m_bmono && !right_data.frame.empty()) 
    {
      img_data = ImageDataConvert(right_data);
      m_padapter->UpdateRightImage(img_data);
    }

    SImuData imu_data;
    for (auto &&data : motion_datas)
    {
      imu_data = IMUDataConvert(data);
      m_padapter->UpdateIMU(imu_data);

      if(imu_data.flag != 0)
      {
        LOG(ERROR) << "IMU FLAG: " << imu_data.flag;
      }
    }
    
    // cv::Mat img;
    // cv::hconcat(left_data.frame, right_data.frame, img);
    
    // SImgData img_left_right = ModifyImage(img, img_data);
    // painter.DrawImgData(img_left_right);
    // if (imu_data.timestamp > 0)
    // {
    //   painter.DrawImuData(img_left_right.frame, imu_data);
    // }

    // cv::imshow("frame", img_left_right.frame);
    // cv::waitKey(1);

    // std::chrono::milliseconds dura(2);
    // std::this_thread::sleep_for(dura);
  }
}

SImgData CMyntS1030Driver::ImageDataConvert(const api::StreamData &src)
{
  SImgData dst;
  src.frame.copyTo(dst.frame);
  dst.frame_id = src.frame_id; //or src.img.frame_id
  dst.timestamp = src.img->timestamp;
  dst.exposure_time = src.img->exposure_time;
  dst.is_ets = src.img->is_ets;
  return dst;
}

// unit of accel from MYNT SDK is g, should be convert to m/s^2,
// unit of gyro from MYNT SDK is deg/sec, should be convert to rad/sec, 
// which the same as sensor_msgs/Imu Message of ROS
SImuData CMyntS1030Driver::IMUDataConvert(const api::MotionData &src)
{
  SImuData dst;
  dst.frame_id = src.imu->frame_id;
  dst.flag= src.imu->flag;
  dst.timestamp = src.imu->timestamp;
  memcpy(dst.accel, src.imu->accel, 3* sizeof(dst.accel[0]));
  memcpy(dst.gyro, src.imu->gyro, 3* sizeof(dst.gyro[0]));
  dst.temperature = src.imu->temperature;

  for (int i = 0; i < 3; i++)
  {
    dst.accel[i] *= G_N;
    dst.gyro[i] /= R2D;
  }
  
  return dst;
}

SImgData CMyntS1030Driver::ModifyImage(const cv::Mat &img, const SImgData &imgData)
{
  SImgData new_image(imgData);
  img.copyTo(new_image.frame);
  return new_image;
}
