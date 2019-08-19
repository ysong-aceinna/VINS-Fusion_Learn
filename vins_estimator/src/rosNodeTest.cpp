/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "estimator/estimator.h"
#include "estimator/parameters.h"
#include "utility/visualization.h"

Estimator estimator;

queue<sensor_msgs::ImuConstPtr> imu_buf;
queue<sensor_msgs::PointCloudConstPtr> feature_buf;
queue<sensor_msgs::ImageConstPtr> img0_buf;
queue<sensor_msgs::ImageConstPtr> img1_buf;
std::mutex m_buf;

/////////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <random>

#define real float

real* accel_noise = NULL;
real* gyro_noise = NULL;
unsigned int noise_count = 0;

struct NoiseModel 
{
    real gyro_noise_mean;
    real gyro_noise_stddev;
    real accel_noise_mean;
    real accel_noise_stddev;
};

// need to be set:
bool add_noise = false;
const unsigned int NOISE_BUF_LEN = 12000*3; //200hz, 1分钟，三轴
const unsigned int IDX = 1;
NoiseModel noise_model_array[] = {
    {0.0, 0.177075, 0.0, 0.03937},               //ARW:1.0   VRW:0.2
    {0.0, 0.31749, 0.0, 0.0657858},              //ARW:1.5   VRW:0.3
    {0.0, 0.444997, 0.0, 0.0906458},             //ARW:2.0   VRW:0.4
    {0.0, 1.1682, 0.0, 0.234272},                //ARW:5.0   VRW:1.0
    {0.0, 2.35188, 0.0, 0.706631},               //ARW:10.0  VRW:3.0
    {0.0, 11.7841, 0.0, 4.71397},                //ARW:50.0  VRW:20.0

    {2.0, 0.0, 0.0, 0.0},                       //gyro_bias:2.0 deg/s
    {3.0, 0.0, 0.0, 0.0},                       //gyro_bias:3.0 deg/s
    {0.0, 0.0, 0.3, 0.0},                       //accel_bias:0.3 m/s^2
    {0.0, 0.0, 1.5, 0.0},                       //accel_bias:1.5 m/s^2
    {0.0, 0.0, 3.0, 0.0},                       //accel_bias:3.0 m/s^2

    {0.0, 0, 0.0, 0}                //ARW:0.66  VRW:0.11 (ADIS14468)
};

real* GenerateGaussianWhiteNoise(unsigned int buflen, real mean, real stddev);

/////////////////////////////////////////////////////////////////////////////////
void img0_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img0_buf.push(img_msg);
    m_buf.unlock();
}

void img1_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img1_buf.push(img_msg);
    m_buf.unlock();
}


cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat img = ptr->image.clone();//SONG:深拷贝。
    return img;
}
//SONG:对于stereo camera，根据时间戳同步两个camera的frame
// extract images with same timestamp from two topics
void sync_process()
{
    while(1)
    {
        if(STEREO)
        {
            cv::Mat image0, image1;
            std_msgs::Header header;
            double time = 0;
            m_buf.lock();
            if (!img0_buf.empty() && !img1_buf.empty())
            {
                double time0 = img0_buf.front()->header.stamp.toSec();
                double time1 = img1_buf.front()->header.stamp.toSec();
                // 0.003s sync tolerance
                if(time0 < time1 - 0.003)//SONG:舍弃time0与time1间隔大于0.003秒的frame。
                {
                    img0_buf.pop();
                    printf("throw img0\n");
                }
                else if(time0 > time1 + 0.003)
                {
                    img1_buf.pop();
                    printf("throw img1\n");
                }
                else
                {
                    time = img0_buf.front()->header.stamp.toSec();
                    header = img0_buf.front()->header;
                    image0 = getImageFromMsg(img0_buf.front());
                    img0_buf.pop();
                    image1 = getImageFromMsg(img1_buf.front());
                    img1_buf.pop();
                    //printf("find img0 and img1\n");
                }
            }
            m_buf.unlock();
            if(!image0.empty())
                estimator.inputImage(time, image0, image1);
        }
        else
        {
            cv::Mat image;
            std_msgs::Header header;
            double time = 0;
            m_buf.lock();
            if(!img0_buf.empty())
            {
                time = img0_buf.front()->header.stamp.toSec();
                header = img0_buf.front()->header;
                image = getImageFromMsg(img0_buf.front());
                img0_buf.pop();
            }
            m_buf.unlock();
            if(!image.empty())
                estimator.inputImage(time, image);
        }

        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}


void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    double t = imu_msg->header.stamp.toSec();
    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;

    //add noise to IMU
    if(add_noise)
    {
        dx += accel_noise[noise_count++];
        dy += accel_noise[noise_count++];
        dz += accel_noise[noise_count++];
        noise_count -= 3;
        rx += gyro_noise[noise_count++];
        ry += gyro_noise[noise_count++];
        rz += gyro_noise[noise_count++];

        if(noise_count >= NOISE_BUF_LEN-3)
        {
            noise_count = 0;
        }
    }

    Vector3d acc(dx, dy, dz);
    Vector3d gyr(rx, ry, rz);

    // Vector3d acc(dz, -1*dy, dx);
    // Vector3d gyr(rz, -1*ry, rx);

    estimator.inputIMU(t, acc, gyr);
    return;
}


void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg)
{
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
    for (unsigned int i = 0; i < feature_msg->points.size(); i++)
    {
        int feature_id = feature_msg->channels[0].values[i];
        int camera_id = feature_msg->channels[1].values[i];
        double x = feature_msg->points[i].x;
        double y = feature_msg->points[i].y;
        double z = feature_msg->points[i].z;
        double p_u = feature_msg->channels[2].values[i];
        double p_v = feature_msg->channels[3].values[i];
        double velocity_x = feature_msg->channels[4].values[i];
        double velocity_y = feature_msg->channels[5].values[i];
        if(feature_msg->channels.size() > 5)
        {
            double gx = feature_msg->channels[6].values[i];
            double gy = feature_msg->channels[7].values[i];
            double gz = feature_msg->channels[8].values[i];
            pts_gt[feature_id] = Eigen::Vector3d(gx, gy, gz);
            //printf("receive pts gt %d %f %f %f\n", feature_id, gx, gy, gz);
        }
        ROS_ASSERT(z == 1);
        Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
        xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
        featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
    }
    double t = feature_msg->header.stamp.toSec();
    estimator.inputFeature(t, featureFrame);
    return;
}

void restart_callback(const std_msgs::BoolConstPtr &restart_msg)
{
    if (restart_msg->data == true)
    {
        ROS_WARN("restart the estimator!");
        estimator.clearState();
        estimator.setParameter();
    }
    return;
}

void imu_switch_callback(const std_msgs::BoolConstPtr &switch_msg)
{
    if (switch_msg->data == true)
    {
        //ROS_WARN("use IMU!");
        estimator.changeSensorType(1, STEREO);
    }
    else
    {
        //ROS_WARN("disable IMU!");
        estimator.changeSensorType(0, STEREO);
    }
    return;
}

void cam_switch_callback(const std_msgs::BoolConstPtr &switch_msg)
{
    if (switch_msg->data == true)
    {
        //ROS_WARN("use stereo!");
        estimator.changeSensorType(USE_IMU, 1);
    }
    else
    {
        //ROS_WARN("use mono camera (left)!");
        estimator.changeSensorType(USE_IMU, 0);
    }
    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vins_estimator");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    gyro_noise = GenerateGaussianWhiteNoise(NOISE_BUF_LEN, noise_model_array[IDX].gyro_noise_mean, noise_model_array[IDX].gyro_noise_stddev);
    cout << "!****************!" << endl;
    accel_noise = GenerateGaussianWhiteNoise(NOISE_BUF_LEN, noise_model_array[IDX].accel_noise_mean, noise_model_array[IDX].accel_noise_stddev);

    if(argc != 2)
    {
        printf("please intput: rosrun vins vins_node [config file] \n"
               "for example: rosrun vins vins_node "
               "~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml \n");
        return 1;
    }

    string config_file = argv[1];
    printf("config_file: %s\n", argv[1]);

    readParameters(config_file);//SONG:从配置文件读取配置参数，并赋给全局变量
    estimator.setParameter();

#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif

    ROS_WARN("waiting for image and imu...");

    registerPub(n);

    ros::Subscriber sub_imu = n.subscribe(IMU_TOPIC, 2000, imu_callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_feature = n.subscribe("/feature_tracker/feature", 2000, feature_callback);
    ros::Subscriber sub_img0 = n.subscribe(IMAGE0_TOPIC, 100, img0_callback);
    ros::Subscriber sub_img1 = n.subscribe(IMAGE1_TOPIC, 100, img1_callback);
    ros::Subscriber sub_restart = n.subscribe("/vins_restart", 100, restart_callback);
    ros::Subscriber sub_imu_switch = n.subscribe("/vins_imu_switch", 100, imu_switch_callback);
    ros::Subscriber sub_cam_switch = n.subscribe("/vins_cam_switch", 100, cam_switch_callback);

    std::thread sync_thread{sync_process};
    ros::spin();

    return 0;
}

//ref:https://stackoverflow.com/questions/32889309/adding-gaussian-noise
real* GenerateGaussianWhiteNoise(unsigned int buflen, real mean, real stddev)
{
    real* noise = new real[buflen];
    memset(noise, 0, buflen*sizeof(real));
    // construct a trivial random generator engine from a time-based seed:
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator (seed);
    std::normal_distribution<real> dist(mean, stddev);
    for (unsigned int i = 0; i < buflen; i++)
    {
        noise[i] = dist(generator);
    }
    
    cout << "mean:" << mean << "  stddev:" << stddev << endl;
    for(int i = 0; i < MIN(10,buflen); i++)
    {
        cout << noise[i] << endl;
    }

    return noise;
}
