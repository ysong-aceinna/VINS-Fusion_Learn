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
#include <config.h>
#include "estimator/parameters.h"

#ifdef ENABLE_MYNT_SDK
#include "driver/mynt_S1030.h"
#elif defined (ENABLE_MYNT_ROS)
#include "driver/mynt_S1030_ros.h"
#else  
#endif

int main(int argc, char **argv)
{
#ifdef ENABLE_MYNT_SDK
    LOG(WARNING) << "ENABLE_MYNT_SDK";
    CDriverBase* pdriver = new CMyntS1030Driver();
#elif defined (ENABLE_MYNT_ROS)
    LOG(WARNING) << "ENABLE_MYNT_ROS";
    CDriverBase* pdriver = new CMyntS1030ROSDriver();
#else  
#endif

    if(argc < 2)
    {
        cout << "please intput: rosrun vins vins_node [config file] \n"
               "for example: rosrun vins vins_node "
               "~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml \n"
               << endl;
        return 1;
    }
    string config_file = argv[1];
    LOG(INFO) << "config_file: " << argv[1];
    readParameters(config_file);//SONG:从配置文件读取配置参数，并赋给全局变量 !!!!! 改为单态

    pdriver->ReadParameters(config_file);
    if(! pdriver->Init(argc, argv))
    {
        LOG(ERROR) << "Driver initialization failed!";
        exit(EXIT_FAILURE);
    }

    CAdapter* padapter = new CAdapter();
    pdriver->AddListener(padapter);
    pdriver->Start();

#ifdef ENABLE_MYNT_SDK
    while (1);
#elif defined (ENABLE_MYNT_ROS)
#else  
#endif
    pdriver->Stop();

/*   test restart driver.
    SAFEDELETE(pdriver);
    SAFEDELETE(padapter);
    padapter = new CAdapter();
#ifdef ENABLE_MYNT_SDK
    CDriverBase* pdriver = new CMyntS1030Driver();
#elif defined (ENABLE_MYNT_ROS)
    CDriverBase* pdriver = new CMyntS1030ROSDriver();
#else  
#endif  
    pdriver->ReadParameters(config_file);
    pdriver->AddListener(padapter);
    pdriver->Init(argc, argv);
    pdriver->Start();
    sleep(2000000);
    pdriver->Stop();
*/
    return 0;
}
