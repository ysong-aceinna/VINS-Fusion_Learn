/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "parameters.h"

double INIT_DEPTH;
double MIN_PARALLAX;
double ACC_N, ACC_W;
double GYR_N, GYR_W;

std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;

Eigen::Matrix3d R_IMU2Body;

Eigen::Vector3d G{0.0, 0.0, 9.8};

double BIAS_ACC_THRESHOLD;
double BIAS_GYR_THRESHOLD;
double SOLVER_TIME;
int NUM_ITERATIONS;
int ESTIMATE_EXTRINSIC;
int ESTIMATE_TD;
int ROLLING_SHUTTER;
std::string EX_CALIB_RESULT_PATH;
std::string VINS_RESULT_PATH;
std::string OUTPUT_FOLDER;
std::string IMU_TOPIC;
int ROW, COL;
double TD;
int NUM_OF_CAM;
int STEREO;
int USE_IMU;
int MULTIPLE_THREAD;
map<int, Eigen::Vector3d> pts_gt;
std::string IMAGE0_TOPIC, IMAGE1_TOPIC;
std::string FISHEYE_MASK;
std::vector<std::string> CAM_NAMES;
int MAX_CNT;
int MIN_DIST;
double F_THRESHOLD;
int SHOW_TRACK;
int FLOW_BACK;
int B_ADD_EXTRA_NOISE;
int EXTRA_NOISE_IDX;

//SONG:从配置文件读取配置参数，并赋给全局变量
void readParameters(std::string config_file)
{
    FILE *fh = fopen(config_file.c_str(),"r");
    if(fh == NULL){
        LOG(ERROR) << "config_file dosen't exist; wrong config_file path";
        return;          
    }
    fclose(fh);

    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    fsSettings["image0_topic"] >> IMAGE0_TOPIC;
    fsSettings["image1_topic"] >> IMAGE1_TOPIC;
    MAX_CNT = fsSettings["max_cnt"];
    MIN_DIST = fsSettings["min_dist"];
    F_THRESHOLD = fsSettings["F_threshold"];
    SHOW_TRACK = fsSettings["show_track"];
    FLOW_BACK = fsSettings["flow_back"];

    MULTIPLE_THREAD = fsSettings["multiple_thread"];

    USE_IMU = fsSettings["imu"];
    printf("USE_IMU: %d\n", USE_IMU);
    if(USE_IMU)
    {
        fsSettings["imu_topic"] >> IMU_TOPIC;
        printf("IMU_TOPIC: %s\n", IMU_TOPIC.c_str());
        ACC_N = fsSettings["acc_n"];
        ACC_W = fsSettings["acc_w"];
        GYR_N = fsSettings["gyr_n"];
        GYR_W = fsSettings["gyr_w"];
        G.z() = fsSettings["g_norm"];
    }

    SOLVER_TIME = fsSettings["max_solver_time"];
    NUM_ITERATIONS = fsSettings["max_num_iterations"];
    MIN_PARALLAX = fsSettings["keyframe_parallax"];
    MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;

    fsSettings["output_path"] >> OUTPUT_FOLDER;
    VINS_RESULT_PATH = OUTPUT_FOLDER + "/vio.csv";
    std::cout << "result path " << VINS_RESULT_PATH << std::endl;
    std::ofstream fout(VINS_RESULT_PATH, std::ios::out);
    fout.close();

    //SONG:
    cv::Mat cv_T_imu2body;
    fsSettings["IMU_T_body"] >> cv_T_imu2body;
    Eigen::Matrix4d T_imu2body;
    cv::cv2eigen(cv_T_imu2body, T_imu2body); 
    R_IMU2Body = T_imu2body.block<3, 3>(0, 0);
    std::cout << "T_IMU_T_body:" << std::endl << T_imu2body << std::endl;

    ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];
    if (ESTIMATE_EXTRINSIC == 2)
    {
        LOG(WARNING) << "have no prior about extrinsic param, calibrate extrinsic param";
        RIC.push_back(Eigen::Matrix3d::Identity());
        TIC.push_back(Eigen::Vector3d::Zero());
        EX_CALIB_RESULT_PATH = OUTPUT_FOLDER + "/extrinsic_parameter.csv";
    }
    else 
    {
        if ( ESTIMATE_EXTRINSIC == 1)
        {
            cout << " Optimize extrinsic param around initial guess!" << endl;
            EX_CALIB_RESULT_PATH = OUTPUT_FOLDER + "/extrinsic_parameter.csv";
        }
        if (ESTIMATE_EXTRINSIC == 0)
            cout << " fix extrinsic param " << endl;

        cv::Mat cv_T;
        fsSettings["body_T_cam0"] >> cv_T;
        Eigen::Matrix4d T;
        cv::cv2eigen(cv_T, T); //SONG:opencv与eigen矩阵转换。

        //SONG: Correct body_T_cam0 by T_imu2body
        std::cout << "body_T_cam0:" << std::endl << T << std::endl;
        T = T_imu2body * T;
        std::cout << "body_T_cam0 (post):" << std::endl << T << std::endl;

        RIC.push_back(T.block<3, 3>(0, 0));//SONG: .block: 矩阵块操作,取左上的3x3部分，即取R
        TIC.push_back(T.block<3, 1>(0, 3));//SONG: .block: 矩阵块操作,取右上的3x1部分，即取T
    } 
    
    NUM_OF_CAM = fsSettings["num_of_cam"];
    printf("camera number %d\n", NUM_OF_CAM);

    if(NUM_OF_CAM != 1 && NUM_OF_CAM != 2)
    {
        printf("num_of_cam should be 1 or 2\n");
        assert(0);
    }


    int pn = config_file.find_last_of('/');
    std::string configPath = config_file.substr(0, pn);
    
    std::string cam0Calib;
    fsSettings["cam0_calib"] >> cam0Calib;
    std::string cam0Path = configPath + "/" + cam0Calib;
    CAM_NAMES.push_back(cam0Path);

    if(NUM_OF_CAM == 2)
    {
        STEREO = 1;
        std::string cam1Calib;
        fsSettings["cam1_calib"] >> cam1Calib;
        std::string cam1Path = configPath + "/" + cam1Calib; 
        //printf("%s cam1 path\n", cam1Path.c_str() );
        CAM_NAMES.push_back(cam1Path);
        
        cv::Mat cv_T;
        fsSettings["body_T_cam1"] >> cv_T;
        Eigen::Matrix4d T;
        cv::cv2eigen(cv_T, T);

        //SONG: Correct body_T_cam1 by T_imu2body
        std::cout << "body_T_cam1:" << std::endl << T << std::endl;
        T = T_imu2body * T;
        std::cout << "body_T_cam1 (post):" << std::endl << T << std::endl;

        RIC.push_back(T.block<3, 3>(0, 0));
        TIC.push_back(T.block<3, 1>(0, 3));
    }

    INIT_DEPTH = 5.0;
    BIAS_ACC_THRESHOLD = 0.1;
    BIAS_GYR_THRESHOLD = 0.1;

    TD = fsSettings["td"];
    ESTIMATE_TD = fsSettings["estimate_td"];
    if (ESTIMATE_TD)
        cout << "Unsynchronized sensors, online estimate time offset, initial td: " << TD << endl;
    else
        cout << "Synchronized sensors, fix time offset: " << TD << endl;

    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    cout << "ROW: " << ROW << " ,COL: " << COL << endl;

    if(!USE_IMU)
    {
        ESTIMATE_EXTRINSIC = 0;
        ESTIMATE_TD = 0;
        printf("no imu, fix extrinsic param; no time offset calibration\n");
    }

    B_ADD_EXTRA_NOISE = fsSettings["b_add_extra_noise"];
    LOG(WARNING) << "Add extra noise to IMU:" << B_ADD_EXTRA_NOISE;

    if(1 == B_ADD_EXTRA_NOISE)
    {
        EXTRA_NOISE_IDX = fsSettings["extra_noise_idx"];
        cout << "extra_noise_idx:" << EXTRA_NOISE_IDX << endl;    
    }
    else if(0 == B_ADD_EXTRA_NOISE)
    {
        ;// do not add noise to imu.
    }
    else
    {
        cout << "Error: b_add_extra_noise should be set 0 or 1." << endl;
    }

    fsSettings.release();
}
