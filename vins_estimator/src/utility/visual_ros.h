/*******************************************************
 * Copyright (C) 2019
 *
 * Author: SongYang (ysong@aceinna.com)
 *******************************************************/
#pragma once

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include "../estimator/parameters.h" //@@@@@
#include "CameraPoseVisualization.h"
#include "visual_base.h"

// using namespace ros; @@@@@
// using namespace Eigen;


class CVisualROS : public CVisualBase
{
private:
    ros::Publisher pub_latest_odometry;
    ros::Publisher pub_path;
    ros::Publisher pub_odometry;
    ros::Publisher pub_point_cloud;
    ros::Publisher pub_margin_cloud;
    ros::Publisher pub_key_poses;
    ros::Publisher pub_camera_pose;
    ros::Publisher pub_camera_pose_visual;
    ros::Publisher pub_keyframe_pose;
    ros::Publisher pub_keyframe_point;
    ros::Publisher pub_extrinsic;
    ros::Publisher pub_image_track;

    // ros::Publisher pub_pose;
    // ros::Publisher pub_cloud;
    // ros::Publisher pub_map;
    // ros::Publisher pub_ref_pose;
    // ros::Publisher pub_cur_pose;
    // ros::Publisher pub_key;
    // ros::Publisher pub_pose_graph;
    nav_msgs::Path path;

    CameraPoseVisualization cameraposevisual;    
    double sum_of_path;
    Eigen::Vector3d last_path;
    size_t pub_counter;

public:
    CVisualROS();
    virtual ~CVisualROS();

    virtual void Init();
    virtual void ShowTrackImage(cv::Mat track_img, double t);
    virtual void ShowLatestOdometry(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, const Eigen::Vector3d &V, double t);
    virtual void ShowResults(const Estimator &estimator, double timestamp);

    void registerPub();
    void pubLatestOdometry(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, const Eigen::Vector3d &V, double t);
    void pubTrackImage(const cv::Mat &imgTrack, const double t);

    void printStatistics(const Estimator &estimator, double t);
    void pubOdometry(const Estimator &estimator, const std_msgs::Header &header);
    // void pubInitialGuess(const Estimator &estimator, const std_msgs::Header &header);
    void pubKeyPoses(const Estimator &estimator, const std_msgs::Header &header);
    void pubCameraPose(const Estimator &estimator, const std_msgs::Header &header);
    void pubPointCloud(const Estimator &estimator, const std_msgs::Header &header);
    void pubTF(const Estimator &estimator, const std_msgs::Header &header);
    void pubKeyframe(const Estimator &estimator);
    // void pubRelocalization(const Estimator &estimator);
    // void pubCar(const Estimator & estimator, const std_msgs::Header &header);
};

