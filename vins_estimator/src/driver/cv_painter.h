/*******************************************************
 * Copyright (C) 2019
 *
 * Author: SongYang (ysong@aceinna.com)
 *******************************************************/
#pragma once

#include <string>
#include <opencv2/core/core.hpp>
#include "struct.h"

class CVPainter {
 public:
  typedef enum Gravity {
    TOP_LEFT,
    TOP_RIGHT,
    BOTTOM_LEFT,
    BOTTOM_RIGHT
  } gravity_t;

  explicit CVPainter();
  ~CVPainter();

  cv::Rect DrawSize(const cv::Mat &img, const gravity_t &gravity = TOP_LEFT);
  
  cv::Rect DrawImgData(
      const SImgData &data,
      const gravity_t &gravity = TOP_LEFT);

  cv::Rect DrawImuData(
      const cv::Mat &img,
      const SImuData& data,
      const gravity_t& gravity = TOP_RIGHT);

  cv::Rect DrawText(
      const cv::Mat &img, const std::string &text,
      const gravity_t &gravity = TOP_LEFT, const int &margin = 5,
      const int &offset_x = 0, const int &offset_y = 0);
};

