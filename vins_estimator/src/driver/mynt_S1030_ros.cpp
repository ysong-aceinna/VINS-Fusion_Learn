/*******************************************************
 * Copyright (C) 2019
 *
 * Author: SongYang (ysong@aceinna.com)
 *******************************************************/
#include "mynt_S1030_ros.h"

CMyntS1030ROSDriver::CMyntS1030ROSDriver()
{
    cout << "CMyntS1030ROSDriver:CMyntS1030ROSDriver()" << endl;
}

CMyntS1030ROSDriver::~CMyntS1030ROSDriver()
{
    cout << "CMyntS1030ROSDriver:~CMyntS1030ROSDriver()" << endl;
}

bool CMyntS1030ROSDriver::Init()
{
    return true;
}

bool CMyntS1030ROSDriver::Init(int argc, char **argv)
{
    ros::init(argc, argv, "vins_estimator");
    return true;
}

/*
Start() can't finish until program exit,
because it is blocked by ros::spin();
*/
void CMyntS1030ROSDriver::Start()
{
    LOG(INFO) << "CMyntS1030ROSDriver::Start()";

    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info); //Info  Debug   Warn, Error,Fatal,

    LOG(WARNING) << "waiting for image and imu...";
    ros::Subscriber sub_imu = n.subscribe(m_imu_topic, 2000, &CMyntS1030ROSDriver::imu_callback, this, ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_feature = n.subscribe("/feature_tracker/feature", 2000, &CMyntS1030ROSDriver::feature_callback, this);
    ros::Subscriber sub_img0 = n.subscribe(m_image0_topic, 100, &CMyntS1030ROSDriver::img0_callback, this);
    ros::Subscriber sub_img1 = n.subscribe(m_image1_topic, 100, &CMyntS1030ROSDriver::img1_callback, this);
    ros::Subscriber sub_restart = n.subscribe("/vins_restart", 100, &CMyntS1030ROSDriver::restart_callback, this);
    ros::Subscriber sub_imu_switch = n.subscribe("/vins_imu_switch", 100, &CMyntS1030ROSDriver::imu_switch_callback, this);
    ros::Subscriber sub_cam_switch = n.subscribe("/vins_cam_switch", 100, &CMyntS1030ROSDriver::cam_switch_callback, this);

    ros::spin();
}

void CMyntS1030ROSDriver::Stop()
{
    LOG(INFO) << "CMyntS1030ROSDriver::Stop()";
}

/*
ROS driver doesn't need start a extra thread to get data,
because driver can get data by callback functions such as 
imu_callback, img0_callback...
*/
void CMyntS1030ROSDriver::ThreadGetData()
{
    LOG(INFO) << "CMyntS1030ROSDriver::ThreadGetData()";
}

//http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/Image.html
SImgData CMyntS1030ROSDriver::ImageDataConvert(const sensor_msgs::ImageConstPtr &img_msg)
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
        LOG(INFO) << "img_msg->encoding: 8UC1";
    }
    else
    {
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
    }
    SImgData dst;
    dst.frame = ptr->image.clone();
    dst.frame_id = img_msg->header.seq;
    dst.timestamp = img_msg->header.stamp.toSec();
    /*
    debug info (EuRoC datasets):
      img_msg->encoding: mono8 ,dst.frame.channels: 1 ,dst.frame.depth: 0 ,dst.frame.dims: 2 ,dst.frame.size: 752 x 480
    */
    // ROS_INFO_STREAM("img_msg->encoding: " << img_msg->encoding 
    // << " ,dst.frame.channels: " << dst.frame.channels()
    // << " ,dst.frame.depth: " << dst.frame.depth()
    // << " ,dst.frame.dims: " << dst.frame.dims 
    // << " ,dst.frame.size: " << dst.frame.cols << " x " << dst.frame.rows );

    return dst;
}

SImuData CMyntS1030ROSDriver::IMUDataConvert(const sensor_msgs::ImuConstPtr &imu_msg)
{
    SImuData dst;
    dst.frame_id = imu_msg->header.seq;
    dst.timestamp = imu_msg->header.stamp.toSec();
    dst.accel[0] = imu_msg->linear_acceleration.x;
    dst.accel[1] = imu_msg->linear_acceleration.y;
    dst.accel[2] = imu_msg->linear_acceleration.z;
    dst.gyro[0] = imu_msg->angular_velocity.x;
    dst.gyro[1] = imu_msg->angular_velocity.y;
    dst.gyro[2] = imu_msg->angular_velocity.z;

    /*
    Debug info (EuRoC):
    frame_id: 9523 ,timestamp: 1403636858.996666 ,accel: 9.504278 , -0.473988 , -3.031889 ,gyro: -0.129852 , -0.012566 , 0.034907
    */

    // ROS_INFO_STREAM("frame_id: " << dst.frame_id 
    // << " ,timestamp: " << setiosflags(ios::fixed) << dst.timestamp
    // << " ,accel: " << dst.accel[0] << " , " << dst.accel[1] << " , " << dst.accel[2]
    // << " ,gyro: " << dst.gyro[0] << " , " << dst.gyro[1] << " , " << dst.gyro[2] );



    // Eigen::Vector3d acc(dx, dy, dz);
    // Eigen::Vector3d gyr(rx, ry, rz);

    // acc = R_IMU2Body*acc;
    // gyr = R_IMU2Body*gyr;

    // //SONG:add noise to IMU for simulation.
    // if(B_ADD_EXTRA_NOISE)
    // {
    //     // cout << "accel1," << acc.x() << "," << acc.y() << "," << acc.z() << endl;
    //     // cout << "gyro1," << gyr.x() << "," << gyr.y() << "," << gyr.z() << endl;
    //     acc(0) += simulator.GetAccelNoise();
    //     acc(1) += simulator.GetAccelNoise();
    //     acc(2) += simulator.GetAccelNoise();
    //     gyr(0) += simulator.GetGyroNoise() / (180.0/M_PI);
    //     gyr(1) += simulator.GetGyroNoise() / (180.0/M_PI);
    //     gyr(2) += simulator.GetGyroNoise() / (180.0/M_PI);
    //     // cout << "accel2," << acc.x() << "," << acc.y() << "," << acc.z() << endl;
    //     // cout << "gyro2," << gyr.x() << "," << gyr.y() << "," << gyr.z() << endl;
    // }

    // estimator.inputIMU(t, acc, gyr);
  return dst;
}

// SImgData CMyntS1030ROSDriver::ModifyImage(const cv::Mat img, const SImgData imgData)
// {
//   SImgData new_image(imgData);
//   img.copyTo(new_image.frame);
//   return new_image;
// }

void CMyntS1030ROSDriver::img0_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    SImgData img_data = ImageDataConvert(img_msg);
    m_padapter->UpdateLeftImage(img_data);
    // LOG(INFO) << "image id: " << img_data.frame_id<< " ,time: " << setiosflags(ios::fixed) << img_data.timestamp << " sec"<<endl;
}

void CMyntS1030ROSDriver::img1_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    SImgData img_data = ImageDataConvert(img_msg);
    m_padapter->UpdateRightImage(img_data);
}

//SONG: NOTE! Accelerations should be in m/s^2 (not in g's), 
//and rotational velocity should be in rad/sec
void CMyntS1030ROSDriver::imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    SImuData imu_data = IMUDataConvert(imu_msg);
    m_padapter->UpdateIMU(imu_data);
    // LOG(INFO) << "IMU id: " << imu_data.frame_id<< " ,time: " << setiosflags(ios::fixed) << imu_data.timestamp << " sec"<<endl;
}

//SONG:正常情况下，是在Estimator::inputImage中根据input image得到featureFrame并将其push到featureBuf
//也可由下边函数，在listrn到feature_msg后，由estimator.inputFeature将featureFramepush到featureBuf。
//节省了计算featureFrame的开销。
void CMyntS1030ROSDriver::feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg)
{
#if 0
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
#endif
    return;
}

//SONG:清空当前的检测状态，并重启算法
void CMyntS1030ROSDriver::restart_callback(const std_msgs::BoolConstPtr &restart_msg)
{
#if 0
    if (restart_msg->data == true)
    {
        ROS_WARN("restart the estimator!");
        estimator.clearState();
        estimator.setParameter();
    }
#endif
    return;
}

//SONG: 设置是否使用IMU.
void CMyntS1030ROSDriver::imu_switch_callback(const std_msgs::BoolConstPtr &switch_msg)
{
#if 0
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
#endif
    return;
}

//SONG: 设置使用Mono or Setero cameras.
void CMyntS1030ROSDriver::cam_switch_callback(const std_msgs::BoolConstPtr &switch_msg)
{
#if 0
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
#endif
    return;
}
