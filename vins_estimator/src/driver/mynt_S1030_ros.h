/*******************************************************
 * Copyright (C) 2019
 *
 * Author: SongYang (ysong@aceinna.com)
 *******************************************************/
#pragma once
#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
// #include <std_msgs/Header.h>
// #include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud.h>
// #include <sensor_msgs/image_encodings.h>
// #include <nav_msgs/Path.h>
// #include <nav_msgs/Odometry.h>
// #include <geometry_msgs/PointStamped.h>
// #include <visualization_msgs/Marker.h>
// #include <tf/transform_broadcaster.h>
#include <opencv2/highgui/highgui.hpp>

#include "../utility/visualization.h"

#include "driver.h"
#include "cv_painter.h"

class CMyntS1030ROSDriver : public CDriverBase
{
private:

public:
  CMyntS1030ROSDriver();
  virtual ~CMyntS1030ROSDriver();

  virtual int Init();
  virtual int Init(int argc, char **argv);
  virtual void Start();
  virtual void Stop();
  virtual void ThreadGetData();

  SImgData ImageDataConvert(const sensor_msgs::ImageConstPtr &img_msg);
  SImuData IMUDataConvert(const sensor_msgs::ImuConstPtr &imu_msg);
  // SImgData ModifyImage(const cv::Mat img, const SImgData imgData);
  
private:
  void img0_callback(const sensor_msgs::ImageConstPtr &img_msg);
  void img1_callback(const sensor_msgs::ImageConstPtr &img_msg);
  void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg);
  void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg);
  void restart_callback(const std_msgs::BoolConstPtr &restart_msg);
  void imu_switch_callback(const std_msgs::BoolConstPtr &switch_msg);
  void cam_switch_callback(const std_msgs::BoolConstPtr &switch_msg);

};
