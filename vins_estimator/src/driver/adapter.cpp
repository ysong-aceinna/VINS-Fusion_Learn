/*******************************************************
 * Copyright (C) 2019
 *
 * Author: SongYang (ysong@aceinna.com)
 *******************************************************/
#include"adapter.h"

CAdapter::CAdapter(/* args */)
{
    m_pestimator = new Estimator();    
    thread_sync = std::thread(&CAdapter::sync_process, this);

    if(B_ADD_EXTRA_NOISE)//SONG: add noise to IMU for simulation.
    {
        simulator.SetNoiseType(EXTRA_NOISE_IDX);
        simulator.GenerateNoiseOnGyroAccel();  
    }
    m_pestimator->setParameter(); //SONG:为estimator设置参数，且启动重要线程: Estimator::processMeasurements
}

CAdapter::~CAdapter()
{
    thread_sync.join();
}

void CAdapter::UpdateLeftImage(SImgData& img)
{
    // cv::imshow("Left", img.frame);
    // cv::waitKey(1);
    // LOG(INFO) << "Left frame_id: " << img.frame_id << ",timestamp: " << setiosflags(ios::fixed) << img.timestamp;
    m_buf.lock();
    img0_buf.push(img);
    m_buf.unlock();
}

void CAdapter::UpdateRightImage(SImgData& img)
{
    // cv::imshow("Right", img.frame);
    // cv::waitKey(1);
    // LOG(INFO) << "Right frame_id: " << img.frame_id << ",timestamp: " << setiosflags(ios::fixed) << img.timestamp;
    m_buf.lock();
    img1_buf.push(img);
    m_buf.unlock();
}

void CAdapter::UpdateIMU(SImuData& imu)
{
    // LOG(INFO) << "Imu frame_id: " << imu.frame_id
    //             << ", timestamp: " << imu.timestamp
    //             << ", accel_x: " << imu.accel[0]
    //             << ", accel_y: " << imu.accel[1]
    //             << ", accel_z: " << imu.accel[2]
    //             << ", gyro_x: " << imu.gyro[0]
    //             << ", gyro_y: " << imu.gyro[1]
    //             << ", gyro_z: " << imu.gyro[2]
    //             << ", temperature: " << imu.temperature;

    // LOG(INFO) << "Imu frame_id: " << setiosflags(ios::fixed) << imu.frame_id
    //             << ", timestamp: " << imu.timestamp;

    Eigen::Vector3d acc(imu.accel[0], imu.accel[1], imu.accel[2]);
    Eigen::Vector3d gyr(imu.gyro[0], imu.gyro[1], imu.gyro[2]);

    /*
    Debug info (EuRoC):
    timestamp: 1403636858.996666 ,accel: 9.504278 , -0.473988 , -3.031889 ,gyro: -0.129852 , -0.012566 , 0.034907
    */
    // ROS_INFO_STREAM("timestamp: " << setiosflags(ios::fixed) << imu.timestamp
    // << " ,accel: " << acc[0] << " , " << acc[1] << " , " << acc[2]
    // << " ,gyro: " << gyr[0] << " , " << gyr[1] << " , " << gyr[2] );

    acc = R_IMU2Body*acc;
    gyr = R_IMU2Body*gyr;

    //SONG:add noise to IMU for simulation.
    if(B_ADD_EXTRA_NOISE)
    {
        // cout << "accel1," << acc.x() << "," << acc.y() << "," << acc.z() << endl;
        // cout << "gyro1," << gyr.x() << "," << gyr.y() << "," << gyr.z() << endl;
        acc(0) += simulator.GetAccelNoise();
        acc(1) += simulator.GetAccelNoise();
        acc(2) += simulator.GetAccelNoise();
        gyr(0) += simulator.GetGyroNoise() / (180.0/M_PI);
        gyr(1) += simulator.GetGyroNoise() / (180.0/M_PI);
        gyr(2) += simulator.GetGyroNoise() / (180.0/M_PI);
        // cout << "accel2," << acc.x() << "," << acc.y() << "," << acc.z() << endl;
        // cout << "gyro2," << gyr.x() << "," << gyr.y() << "," << gyr.z() << endl;
    }

    /*
    Debug info (EuRoC):
    timestamp: 1403636858.996666 ,accel: -3.031889 , 0.473988 , 9.504278 ,gyro: 0.034907 , 0.012566 , -0.129852
    */
    // ROS_INFO_STREAM("timestamp: " << setiosflags(ios::fixed) << imu.timestamp
    // << " ,accel: " << acc[0] << " , " << acc[1] << " , " << acc[2]
    // << " ,gyro: " << gyr[0] << " , " << gyr[1] << " , " << gyr[2] );

    m_pestimator->inputIMU(imu.timestamp, acc, gyr);
    return;    
}

//SONG:对于stereo camera，根据时间戳同步两个camera的frame
// extract images with same timestamp from two topics
void CAdapter::sync_process()
{
    while(1)
    {
        if(STEREO)
        {
            cv::Mat image0, image1;
            double time = 0;
            m_buf.lock();
            if (!img0_buf.empty() && !img1_buf.empty())
            {
                double time0 = img0_buf.front().timestamp;
                double time1 = img1_buf.front().timestamp;
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
                    time = img0_buf.front().timestamp;
                    image0 = img0_buf.front().frame;
                    img0_buf.pop();
                    image1 = img1_buf.front().frame;
                    img1_buf.pop();
                    //printf("find img0 and img1\n");
                }
            }
            m_buf.unlock();
            if(!image0.empty())
                m_pestimator->inputImage(time, image0, image1);
        }
        else
        {
            cv::Mat image;
            double time = 0;
            m_buf.lock();
            if(!img0_buf.empty())
            {
                time = img0_buf.front().timestamp;
                image = img0_buf.front().frame;
                img0_buf.pop();
            }
            m_buf.unlock();
            if(!image.empty())
                m_pestimator->inputImage(time, image);
        }

        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);//SONG: C++11 sleep for 2ms
    }
}

