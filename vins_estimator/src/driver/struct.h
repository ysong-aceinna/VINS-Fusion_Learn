/*******************************************************
 * Copyright (C) 2019
 *
 * Author: SongYang (ysong@aceinna.com)
 *******************************************************/
#pragma once

#include <opencv2/opencv.hpp>

//refer to struct ImuData of MYNT SDK as standard imu struct.
struct SImuData {
  /** IMU frame id */
  std::uint32_t frame_id;
  /**
   * IMU accel or gyro flag
   * <p>
   *   0: accel and gyro are both valid<br>
   *   1: accel is valid<br>
   *   2: gyro is valid
   * </p>
   */
  std::uint8_t flag;
  /** Is external time source */
  bool is_ets = false;
  /** IMU timestamp in 1us */
  // std::uint64_t timestamp;
  double timestamp;
  /* Refer to http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html */
  /* Accelerations and rotational velocity should be same as sensor_msgs/Imu Message of ROS.*/
  /* accel unit: m/s^2    gyro unit: rad/s  */
  /** IMU accelerometer data for 3-axis: X, Y, Z.*/ 
  double accel[3];
  /** IMU gyroscope data for 3-axis: X, Y, Z. */
  double gyro[3];
  /** IMU temperature */
  double temperature;

  void Reset() {
    flag = 0;
    timestamp = 0;
    std::fill(accel, accel + 3, 0);
    std::fill(gyro, gyro + 3, 0);
    temperature = 0;
  }

  SImuData() {
    Reset();
  }

  SImuData(const SImuData &other) {
    frame_id = other.frame_id;
    flag = other.flag;
    is_ets = other.is_ets;
    timestamp = other.timestamp;
    memcpy(accel, other.accel, 3* sizeof(accel[0]));
    memcpy(gyro, other.gyro, 3* sizeof(gyro[0]));
    temperature = other.temperature;
  }

  SImuData &operator=(const SImuData &other) {
    frame_id = other.frame_id;
    flag = other.flag;
    is_ets = other.is_ets;
    timestamp = other.timestamp;
    memcpy(accel, other.accel, 3* sizeof(accel[0]));
    memcpy(gyro, other.gyro, 3* sizeof(gyro[0]));
    temperature = other.temperature;
    return *this;
  }

  bool operator==(const SImuData &other) const {
    if (timestamp == other.timestamp && frame_id == other.frame_id) {
      return true;
    }
    return false;
  }  
};

struct SImgData {
  /** Frame. */
  cv::Mat frame;
  /** Image frame id */
  std::uint16_t frame_id;
  /** Image timestamp in 1us */
  // std::uint64_t timestamp;
  double timestamp;
  /** Image exposure time, virtual value in [1, 480] */
  std::uint16_t exposure_time;
  /** Is external time source */
  bool is_ets = false;

  void Reset() {
    frame_id = 0;
    timestamp = 0;
    exposure_time = 0;
    is_ets = false;
  }

  SImgData() {
    Reset();
  }

  SImgData(const SImgData &other) {
    other.frame.copyTo(frame);
    frame_id = other.frame_id;
    timestamp = other.timestamp;
    exposure_time = other.exposure_time;
    is_ets = other.is_ets;
  }
  
  SImgData &operator=(const SImgData &other) {
    other.frame.copyTo(frame);
    frame_id = other.frame_id;
    timestamp = other.timestamp;
    exposure_time = other.exposure_time;
    is_ets = other.is_ets;
    return *this;
  }

  bool operator==(const SImgData &other) const {
    if (!frame.empty() && !other.frame.empty()) {
      return frame_id == other.frame_id &&
             timestamp == other.timestamp;
    }
    return false;
  }

};

struct NoiseModel 
{
    float gyro_noise_mean;
    float gyro_noise_stddev;
    float accel_noise_mean;
    float accel_noise_stddev;
};
