/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "estimator.h"

Estimator::Estimator(): f_manager{Rs}
{
#ifdef ENABLE_MYNT_SDK
    m_pvisual = new CVisualSDK();
#elif defined (ENABLE_MYNT_ROS)
    m_pvisual = new CVisualROS();
#else  
#endif

    LOG(INFO) <<"init begins";
    m_pvisual->Init();
    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        pre_integrations[i] = nullptr;
    }
    tmp_pre_integration = nullptr;
    last_marginalization_info = nullptr;

    initThreadFlag = false;
    clearState();

    if(!CreateLogFiles())
    {
        exit(EXIT_FAILURE);
    }
}

Estimator::~Estimator()
{
    if (MULTIPLE_THREAD)
    {
        processThread.join();
        printf("join thread \n");
    }

    m_fout_imu_bias.close();
    m_fout_linear_acc.close();
}

void Estimator::clearState()
{
    //清空IMU和feature buf缓存的数据。
    mProcess.lock();
    while(!accBuf.empty())
        accBuf.pop();
    while(!gyrBuf.empty())
        gyrBuf.pop();
    while(!featureBuf.empty())
        featureBuf.pop();

    prevTime = -1;
    curTime = 0;
    openExEstimation = 0;
    initP = Eigen::Vector3d(0, 0, 0);
    initR = Eigen::Matrix3d::Identity();
    inputImageCnt = 0;
    initFirstPoseFlag = false;

    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        Rs[i].setIdentity();
        Ps[i].setZero();
        Vs[i].setZero();
        Bas[i].setZero();
        Bgs[i].setZero();
        dt_buf[i].clear();
        linear_acceleration_buf[i].clear();
        angular_velocity_buf[i].clear();

        if (pre_integrations[i] != nullptr)
        {
            delete pre_integrations[i];
            pre_integrations[i] = nullptr;        
        }
    }

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = Vector3d::Zero();
        ric[i] = Matrix3d::Identity();
    }

    first_imu = false,
    sum_of_back = 0;
    sum_of_front = 0;
    frame_count = 0;
    solver_flag = INITIAL;
    initial_timestamp = 0;
    all_image_frame.clear();

    if (tmp_pre_integration != nullptr)
    {
        delete tmp_pre_integration;
        tmp_pre_integration = nullptr;
    }
    if (last_marginalization_info != nullptr)
    {
        delete last_marginalization_info;
        last_marginalization_info = nullptr;
    }
    last_marginalization_parameter_blocks.clear();

    f_manager.clearState();
    failure_occur = 0;
    mProcess.unlock();
}

/*
SONG:设置Estimator相关参数。
注意，本函数涉及的参数只是一部分，很多参数是通过全局变量的方式访问的。
*/
void Estimator::setParameter()
{
    mProcess.lock();
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = TIC[i]; //SONG:cam0到body(IMU)的平移和旋转矩阵。
        ric[i] = RIC[i];
        cout << " exitrinsic cam " << i << endl  << ric[i] << endl << tic[i].transpose() << endl;
    }
    f_manager.setRic(ric);   
    //SONG:视觉约束相关的costfunction.
    //ProjectionTwoFrameOneCamFactor:对单目相机，相邻两个视频帧，对于同一个feature的约束。崔华坤[3.4节]
    //ProjectionTwoFrameTwoCamFactor:对双目相机，相邻两个视频帧，对两个feature的约束
    //ProjectionOneFrameTwoCamFactor:对双目相机，一个视频帧，对两个feature的约束
    ProjectionTwoFrameOneCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity(); //SONG:Identity()单位矩阵
    ProjectionTwoFrameTwoCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    ProjectionOneFrameTwoCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    td = TD;  //SONG:取自yaml的td 和 g_norm。
    g = G;
    cout << "set g " << g.transpose() << endl; //SONG:矩阵的转置
    featureTracker.readIntrinsicParameter(CAM_NAMES); //SONG:取自yaml的cam0_calib，即读取摄像机的mei/pinhole/equ校准参数。

    std::cout << "MULTIPLE_THREAD is " << MULTIPLE_THREAD << '\n';
    if (MULTIPLE_THREAD && !initThreadFlag)
    {
        initThreadFlag = true;
        processThread = std::thread(&Estimator::processMeasurements, this);//SONG:线程创建就立即执行了。
    }
    mProcess.unlock();
}

//changeSensorType似的程序在运行期间可以在Mono+IMU, Stereo+IMU, Stereo三种模式间切换，这个功能一般不用。
void Estimator::changeSensorType(int use_imu, int use_stereo)
{
    bool restart = false;
    mProcess.lock();
    if(!use_imu && !use_stereo)
        printf("at least use two sensors! \n");
    else
    {
        if(USE_IMU != use_imu)
        {
            USE_IMU = use_imu;
            if(USE_IMU)
            {
                // reuse imu; restart system
                restart = true;
            }
            else
            {
                if (last_marginalization_info != nullptr)
                    delete last_marginalization_info;

                tmp_pre_integration = nullptr;
                last_marginalization_info = nullptr;
                last_marginalization_parameter_blocks.clear();
            }
        }
        
        STEREO = use_stereo;
        printf("use imu %d use stereo %d\n", USE_IMU, STEREO);
    }
    mProcess.unlock();
    if(restart)
    {
        clearState();
        setParameter();
    }
}

//SONG:检测trackImage，放入featureBuf。featureBuf会在processMeasurements中被处理。
void Estimator::inputImage(double t, const cv::Mat &_img, const cv::Mat &_img1)
{
    inputImageCnt++;

    //为使initFirstIMUPose中有足够的imu数据计算初始姿态,应跳过初始的jump_frames_cnt帧图像.
    //initFirstIMUPose中IMU数据个数约为: 
    // 1. 若jump_frames_cnt为奇数: (jump_frames_cnt+1) *(IMU频率/图像频率)
    // 2. 若jump_frames_cnt为偶数, jump_frames_cnt*(IMU频率/图像频率)
    // 举例: 当jump_frames_cnt为2时,如image为20hz,对应20个imu数据;
    //      当jump_frames_cnt为2时,如image为10hz,对应40个imu数据;
    const int jump_frames_cnt = 2; 
    if(inputImageCnt < jump_frames_cnt ) return;

    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
    TicToc featureTrackerTime;

    //SONG:featureFrame是特征帧（坐标和速度等特征），而不是光流法里的特征点，不要混淆。
    if(_img1.empty())
        featureFrame = featureTracker.trackImage(t, _img);
    else
        featureFrame = featureTracker.trackImage(t, _img, _img1);
    // LOG(INFO) << "featureTracker time: " << featureTrackerTime.toc();

    //SONG:imgTrack是在输入图像上绘制红、绿、蓝特征点的图像。
    if (SHOW_TRACK)
    {
        cv::Mat imgTrack = featureTracker.getTrackImage();
        m_pvisual->ShowTrackImage(imgTrack, t);
    }
    
    if(MULTIPLE_THREAD)  
    {     
        if(inputImageCnt % 2 == 0)    //SONG:若采用多线程，只将偶数帧的featureFrame保存到featureBuf
        {
            mBuf.lock();
            featureBuf.push(make_pair(t, featureFrame));
            mBuf.unlock();
        }
    }
    else
    {
        mBuf.lock();
        featureBuf.push(make_pair(t, featureFrame));
        mBuf.unlock();
        TicToc processTime; //SONG:计时。
        processMeasurements();
        printf("process time: %f\n", processTime.toc());
    }
    
}

//SONG:accel和gyro数据放accbuf和gyrbuf；IMU做预计分。
void Estimator::inputIMU(double t, const Vector3d &linearAcceleration, const Vector3d &angularVelocity)
{
    mBuf.lock();
    accBuf.push(make_pair(t, linearAcceleration));
    gyrBuf.push(make_pair(t, angularVelocity));
    //printf("input imu with time %f \n", t);
    mBuf.unlock();

    if (solver_flag == NON_LINEAR) //除了在image到来时得到准确位姿外，还可以在相邻两图像帧间根据IMU递推最新的位姿，和IMU同频率。
    {
        mPropagate.lock();
        //SONG:IMU快速计分求当前的速度和位置。
        // 我觉得这里的fastPredictIMU可以删掉，因为在processImage中又被重复调用了。
        Eigen::Vector3d un_acc = fastPredictIMU(t, linearAcceleration, angularVelocity);
        saveLinearAcc(t, un_acc); //save linear accel.
        m_pvisual->ShowLatestOdometry(latest_P, latest_Q, latest_V, t);
        mPropagate.unlock();
    }
}

void Estimator::inputFeature(double t, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &featureFrame)
{
    mBuf.lock();
    featureBuf.push(make_pair(t, featureFrame));
    mBuf.unlock();

    if(!MULTIPLE_THREAD)
        processMeasurements();
}

//SONG:把从t0到t1期间所有的imu数据放入accVector, gyrVector
bool Estimator::getIMUInterval(double t0, double t1, vector<pair<double, Eigen::Vector3d>> &accVector, 
                                vector<pair<double, Eigen::Vector3d>> &gyrVector)
{
    if(accBuf.empty())
    {
        printf("not receive imu\n");
        return false;
    }
    //printf("get imu from %f %f\n", t0, t1);
    //printf("imu fornt time %f   imu end time %f\n", accBuf.front().first, accBuf.back().first);

    /*
    SONG:将imuBuf中:
       . <=t0 的部分删除；
       . （t0, t1)的部分放到accVector和gyrVector
       . >t1 的部分不动
    */
    if(t1 <= accBuf.back().first)
    {
        while (accBuf.front().first <= t0)//SONG:把早于t0的IMU数据从accBuf和gyrBuf中删除。
        {
            accBuf.pop();
            gyrBuf.pop(); 
        }
        while (accBuf.front().first < t1)
        {
            accVector.push_back(accBuf.front());
            accBuf.pop();//SONG: 用完即删。
            gyrVector.push_back(gyrBuf.front());
            gyrBuf.pop();
        }

        //SONG: 上边的while循环已经把[t0, t1]间的数据都填充进accVector和gyrVector了，
        //下边两行的目的是,临界时间点的一帧IMU数据重用.
        accVector.push_back(accBuf.front()); 
        gyrVector.push_back(gyrBuf.front());
    }
    else
    {
        printf("wait for imu\n");
        return false;
    }
    return true;
}

//SONG:如果imubuf不为空，且相对t有新数据。
bool Estimator::IMUAvailable(double t)
{
    if(!accBuf.empty() && t <= accBuf.back().first)
        return true;
    else
        return false;
}

void Estimator::processMeasurements()
{
    while (1)
    {
        //printf("process measurments\n");
        pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 7, 1> > > > > feature;
        vector<pair<double, Eigen::Vector3d>> accVector, gyrVector;
        if(!featureBuf.empty())
        {
            feature = featureBuf.front();
            //SONG:prevTime, curTime是收到的相邻两帧图像的时间戳。
            //如果认为图像的接收有延时，即图像和IMU数据不同步，则通过td来做时间对齐（同步）。
            curTime = feature.first + td; 
            while(1) //SONG:检查IMU数据可用，可用就直接跳出while，不可用就等5ms继续检查
            {   //SONG:这里应该把!USE_IMU的判断提到while外边，否则有两个问题：
                //1. !USE_IMU会影响效率，如果没有IMU的话，相当于每次都要进while循环。
                //2. 如果没有IMU的话，也就不用判断IMUAvailable了。
                if ((!USE_IMU  || IMUAvailable(feature.first + td)))
                    break;
                else
                {
                    printf("wait for imu ... \n");//SONG:用S1030经常发现有这个提示。但用EuRoC极少出现。
                    if (! MULTIPLE_THREAD)
                        return;
                    std::chrono::milliseconds dura(5);
                    std::this_thread::sleep_for(dura);
                }
            }
            mBuf.lock();
            if(USE_IMU)//SONG:把从prevTime到curTime期间所有的imu数据放入accVector, gyrVector
                getIMUInterval(prevTime, curTime, accVector, gyrVector); 

            featureBuf.pop();
            mBuf.unlock();

            if(USE_IMU)
            {
                if(!initFirstPoseFlag)
                {
                    //SONG: 获取IMU的初始姿态，并赋给Rs[0]。
                    //有个问题：基于加速度计计算初始姿态,此时需要初始时的body要在静止状态，否则得到的初始姿态是不准的。
                    initFirstIMUPose(accVector);
                }
                for(size_t i = 0; i < accVector.size(); i++)
                {
                    double dt;
                    if(i == 0)
                        dt = accVector[i].first - prevTime;
                    else if (i == accVector.size() - 1)
                        dt = curTime - accVector[i - 1].first;
                    else
                        dt = accVector[i].first - accVector[i - 1].first;
                    //SONG:处理IMU数据。
                    processIMU(accVector[i].first, dt, accVector[i].second, gyrVector[i].second);
                }
            }
            mProcess.lock();
            processImage(feature.second, feature.first);//SONG:feature.first是时间(秒)，
            prevTime = curTime;
            m_pvisual->ShowResults(*this, feature.first);
            mProcess.unlock();
        }

        if (! MULTIPLE_THREAD)
            break;

        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

//SONG: 获取IMU的初始姿态，并赋给Rs[0]，并以此结果建立坐标系。
//这里要测试一下,如果不是在车辆静止状态,而是在有加减速的汽车行驶过程中得到的初始姿态,对后边的最终结果有什么影响.
//测试结果：
//1. 在公司院子里，汽车转弯过程中做初始化，对检测结果影响不大
//2. 在马路上转弯时做初始化，结果偏差非常大。
void Estimator::initFirstIMUPose(vector<pair<double, Eigen::Vector3d>> &accVector)
{
    printf("init first imu pose\n");
    initFirstPoseFlag = true;
    //return;
    Eigen::Vector3d averAcc(0, 0, 0);
    int n = (int)accVector.size();
    for(size_t i = 0; i < accVector.size(); i++)
    {
        averAcc = averAcc + accVector[i].second;
    }
    averAcc = averAcc / n; //SONG:求accVector中n组数据的均值。
    printf("averge acc %f %f %f, n:%d\n", averAcc.x(), averAcc.y(), averAcc.z(), n);
    Matrix3d R0 = Utility::g2R(averAcc); //SONG: R0是以余弦矩阵表示的初始姿态。
    double yaw = Utility::R2ypr(R0).x();
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0; //经调试,yaw一直是0,对头.
    Rs[0] = R0;
    cout << "init R0 " << endl << Rs[0] << endl;
    //Vs[0] = Vector3d(5, 0, 0);
}

//SONG：全局搜了下，initFirstPose及initP，initR从未用过。
//估计应该是用于外界已知初始姿态，调用此接口可做初始姿态的初始化。
//这个在做融合算法时会用到，比如openRTK 先获取一个初始姿态给VINS，实现对齐。
void Estimator::initFirstPose(Eigen::Vector3d p, Eigen::Matrix3d r)
{
    Ps[0] = p;
    Rs[0] = r;
    initP = p;
    initR = r;
}

//SONG:
// 1. 类似fastPredictIMU,递推速度和位置，不同的是fastPredictIMU是递推到latest_P，而这里是递推到Ps。
// 2. 构造pre_integrations，它会计算雅克比矩阵、残差的协方差矩阵等参数，这些参数优化时会用到。
void Estimator::processIMU(double t, double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity)
{
    if (!first_imu)
    {
        first_imu = true;
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
    }

    if (!pre_integrations[frame_count])
    {
        pre_integrations[frame_count] = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};//Bas,Bgs由optimization得到。
    }
    if (frame_count != 0)//为什么滑动窗口的第一帧不需要做下边的预计分和递推？
    {
        pre_integrations[frame_count]->push_back(dt, linear_acceleration, angular_velocity);

        /*
        下边对tmp_pre_integration又做了一次IMU的预计分，是对计算资源的浪费。两种改进方法:
        1. 在processIMU中做修改，将下边pre_integrations[frame_count]->push_back的结果通过拷贝构造函数赋值给tmp_pre_integration，缺点是每做一次预计分就要赋值一次;
        2. 在processImage中修改，将pre_integrations[frame_count]->push_back的结果通过拷贝构造函数赋值给tmp_pre_integration，相对方法1的优点是，只在使用tmp_pre_integration前做一次赋值，
        */

        //if(solver_flag != NON_LINEAR)
            tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity);

        dt_buf[frame_count].push_back(dt);
        linear_acceleration_buf[frame_count].push_back(linear_acceleration);
        angular_velocity_buf[frame_count].push_back(angular_velocity);

        //下边为中值积分。本套代码中有多个地方都在做中值积分，目的和区别是什么呢？
        //下边中值积分得到的Rs, Ps, Vs为optimization提供优化的初值。
        int j = frame_count;
        Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - g;
        Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[j];
        Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
        Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - g;
        Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
        Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc;
        Vs[j] += dt * un_acc;
    }
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity; 
}

/*
对应VINS-MONO论文中的图7.
为了处理一些悬停的case,引入了一个two-way marginalization, 
简单来说就是:如果倒数第二帧是关键帧, 则将最旧的pose移出sliding window, 
将最旧帧相关联的视觉和惯性数据边缘化掉，也就是MARGIN_OLD，
作为一部分先验值,如果倒数第二帧不是关键帧, 则将倒数第二帧pose移出sliding window,
将倒数第二帧的视觉观测值直接舍弃，保留相关联的IMU数据， 也就是MARGIN_NEW。
选取关键帧的策略是视差足够大，在悬停等运动较小的情况下, 会频繁的MARGIN_NEW, 
这样也就保留了那些比较旧但是视差比较大的pose. 这种情况如果一直MARGIN_OLD的话, 
视觉约束不够强, 状态估计会受IMU积分误差影响, 具有较大的累积误差。
*/
void Estimator::processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, const double header)
{
    DLOG(INFO) << "new image coming ------------------------------------------";
    DLOG(INFO) << "Adding feature points " << image.size();
    //检查视差：足够大，则将窗口中最老的一帧边缘化掉，否则将第二新的帧边缘化掉。
    if (f_manager.addFeatureCheckParallax(frame_count, image, td))
    {
        marginalization_flag = MARGIN_OLD;
        //printf("keyframe\n");
    }
    else
    {
        marginalization_flag = MARGIN_SECOND_NEW;
        //printf("non-keyframe\n");
    }

    DLOG(INFO) << (marginalization_flag ? "Non-keyframe" : "Keyframe") ;
    DLOG(INFO) << "Solving " << frame_count;
    DLOG(INFO) << "number of feature: " << f_manager.getFeatureCount();
    Headers[frame_count] = header;

    ImageFrame imageframe(image, header); //image实际上是特征点，命名有歧义。
    imageframe.pre_integration = tmp_pre_integration;
    all_image_frame.insert(make_pair(header, imageframe));
    tmp_pre_integration = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};

    if(ESTIMATE_EXTRINSIC == 2)//如果完全不知道T_camera_IMU,则使用CalibrationExRotation计算一个大概的T，后边再通过优化的方式估算一个更准确的T.
    {
        LOG(INFO) << "calibrating extrinsic param, rotation movement is needed";
        if (frame_count != 0)
        {
            vector<pair<Vector3d, Vector3d>> corres = f_manager.getCorresponding(frame_count - 1, frame_count);
            Matrix3d calib_ric;
            if (initial_ex_rotation.CalibrationExRotation(corres, pre_integrations[frame_count]->delta_q, calib_ric))
            {
                LOG(WARNING) << "initial extrinsic rotation calib success";
                LOG(WARNING) << "initial extrinsic rotation: " << endl << calib_ric;
                ric[0] = calib_ric;
                RIC[0] = calib_ric;
                ESTIMATE_EXTRINSIC = 1;
            }
        }
    }

    //SONG:初始化:
    //  1. 当滑动窗口buf全部被填满后(frame_count == WINDOW_SIZE), 做初始化
    //  2. 初始化成功后,将solver_flag = NON_LINEAR, 并做一次optimization
    if (solver_flag == INITIAL)
    {
        // monocular + IMU initilization
        if (!STEREO && USE_IMU)
        {
            if (frame_count == WINDOW_SIZE) //窗口填满后,开始SFM
            {
                bool result = false;
                if(ESTIMATE_EXTRINSIC != 2 && (header - initial_timestamp) > 0.1)
                {
                    result = initialStructure();
                    initial_timestamp = header;   
                }
                if(result)
                {
                    optimization();
                    updateLatestStates();
                    solver_flag = NON_LINEAR;
                    optimization();//SONG: 为什么再做一次优化呢？
                    slideWindow();
                    LOG(INFO) << "Initialization finish!";
                }
                else
                    slideWindow();
            }
        }

        // stereo + IMU initilization
        if(STEREO && USE_IMU)
        {
            f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);
            f_manager.triangulate(frame_count, Ps, Rs, tic, ric);
            if (frame_count == WINDOW_SIZE)
            {
                map<double, ImageFrame>::iterator frame_it;
                int i = 0;
                for (frame_it = all_image_frame.begin(); frame_it != all_image_frame.end(); frame_it++)
                {
                    frame_it->second.R = Rs[i];
                    frame_it->second.T = Ps[i];
                    i++;
                }
                solveGyroscopeBias(all_image_frame, Bgs);
                for (int i = 0; i <= WINDOW_SIZE; i++)
                {
                    pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);
                }
                optimization();
                updateLatestStates();
                solver_flag = NON_LINEAR;
                slideWindow();
                LOG(INFO) << "Initialization finish!";
            }
        }

        // stereo only initilization
        if(STEREO && !USE_IMU)
        {
            f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);
            f_manager.triangulate(frame_count, Ps, Rs, tic, ric);
            optimization();

            if(frame_count == WINDOW_SIZE)
            {
                optimization();
                updateLatestStates();
                solver_flag = NON_LINEAR;
                slideWindow();
                LOG(INFO) << "Initialization finish!";
            }
        }

        //SONG:滑动窗口buf未被填满前, 用buf[i-1]填充buf[i]
        if(frame_count < WINDOW_SIZE)
        {
            frame_count++;
            int prev_frame = frame_count - 1;
            Ps[frame_count] = Ps[prev_frame];
            Vs[frame_count] = Vs[prev_frame];
            Rs[frame_count] = Rs[prev_frame];
            Bas[frame_count] = Bas[prev_frame];
            Bgs[frame_count] = Bgs[prev_frame];
        }

    }
    else//初始化成功后的处理
    {
        TicToc t_solve;
        if(!USE_IMU)
            f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);
        f_manager.triangulate(frame_count, Ps, Rs, tic, ric);
        optimization();
        set<int> removeIndex;
        outliersRejection(removeIndex);
        f_manager.removeOutlier(removeIndex);
        if (! MULTIPLE_THREAD)
        {
            featureTracker.removeOutliers(removeIndex);
            predictPtsInNextFrame();
        }
            
        DLOG(INFO) << "solver costs: " << t_solve.toc() << "ms";

        //SONG: 失效自检测，自动重启算法。实际上由于failureDetection强制return false，所以下边的if永远没有执行。
        if (failureDetection())
        {
            LOG(WARNING) << "failure detection";
            failure_occur = 1;
            clearState();
            setParameter();
            LOG(WARNING) << "system reboot!";
            return;
        }

        slideWindow();
        f_manager.removeFailures();
        // prepare output of VINS
        key_poses.clear();
        for (int i = 0; i <= WINDOW_SIZE; i++)
            key_poses.push_back(Ps[i]);

        last_R = Rs[WINDOW_SIZE];
        last_P = Ps[WINDOW_SIZE];
        last_R0 = Rs[0];
        last_P0 = Ps[0];
        updateLatestStates();
    }  
}

bool Estimator::initialStructure()
{
    TicToc t_sfm;
    //SONG:检查IMU的运动幅度是不是太小。实践中发现，当image帧率较小时，需要更大的IMU运动幅度。
    //check imu observibility
    {
        map<double, ImageFrame>::iterator frame_it;
        Vector3d sum_g;
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;//pre_integration中记录了预计分的时长sum_dt.
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;//delta_v是预计分中V的增量部分。
            sum_g += tmp_g;
        }
        Vector3d aver_g; //即滑窗内所有tmp_g累加所得的sum_g的均值。
        aver_g = sum_g * 1.0 / ((int)all_image_frame.size() - 1);
        double var = 0;
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);
            //cout << "frame g " << tmp_g.transpose() << endl;
        }
        //计算tmp_g的标准差
        var = sqrt(var / ((int)all_image_frame.size() - 1));
        // LOG(WARNING) << "IMU variation " << var << "!";
        // 根据标准差判断是否移动幅度较小.
        // 期初感觉不太合理，如果是匀速运动，var是0，相机也可能有较大平移并产生视差。
        // 实际上，汽车从静止开始运动并做初始化时，不存在匀速运动的情况。一般是具有明显加速度的。
        if(var < 0.25)
        {
            LOG(INFO) << "IMU excitation not enouth!";
            //return false;
        }
    }
    // global sfm
    Quaterniond Q[frame_count + 1];
    Vector3d T[frame_count + 1];
    map<int, Vector3d> sfm_tracked_points; //三角化出来的 3D 路标点
    vector<SFMFeature> sfm_f;
    for (auto &it_per_id : f_manager.feature)//f_manager.feature：滑窗内所有特征点。
    {
        int imu_j = it_per_id.start_frame - 1;
        //对于一个world系下的被观测点，被滑窗内n个图像观察到并生成n个特征点。
        // tmp_feature 存储的是，一个world系下的被观测点对应的n个特征点的归一化平面坐标，及其在滑窗内的位置。
        //sfm_f存储的是所有特征点在n个窗口内的信息。
        SFMFeature tmp_feature; 
        tmp_feature.state = false;
        tmp_feature.id = it_per_id.feature_id;
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            Vector3d pts_j = it_per_frame.point; //pts_j为归一化平面上的坐标。
            tmp_feature.observation.push_back(make_pair(imu_j, Eigen::Vector2d{pts_j.x(), pts_j.y()}));
        }
        sfm_f.push_back(tmp_feature);
    } 
    Matrix3d relative_R;
    Vector3d relative_T;
    int l; //SONG: 计算出relative_R, relative_T时，WINDOW_SIZE的编号索引。
    // 并不是拿滑窗内所有的视频帧来求解相对位姿，只是用当前帧和参考帧（匹配特征点数较多的关键帧作为参考帧）求解。
    // l就是参考帧在滑动窗口中的索引位置。
    if (!relativePose(relative_R, relative_T, l)) //SONG:根据对极几何，先求F基础矩阵，再求relative_R, relative_T。
    {
        LOG(INFO) << "Not enough features or parallax; Move device around";
        return false;
    }
    GlobalSFM sfm;
    //SONG: 利用对极几何求出的relative_R, relative_T并不准确，
    // 需要在sfm.construct中用Ceres进一步优化，Q, T是优化后的结果。
    if(!sfm.construct(frame_count + 1, Q, T, l,
              relative_R, relative_T,
              sfm_f, sfm_tracked_points))
    {
        DLOG(INFO) << "global SFM failed!";
        marginalization_flag = MARGIN_OLD;
        return false;
    }

    //计算滑窗内，所有帧相对第一个滑窗的R ,t
    //solve pnp for all frame
    map<double, ImageFrame>::iterator frame_it;
    map<int, Vector3d>::iterator it;
    frame_it = all_image_frame.begin( );
    for (int i = 0; frame_it != all_image_frame.end( ); frame_it++)
    {
        // provide initial guess
        cv::Mat r, rvec, t, D, tmp_r;
        if((frame_it->first) == Headers[i])
        {
            frame_it->second.is_key_frame = true;
            frame_it->second.R = Q[i].toRotationMatrix() * RIC[0].transpose();
            frame_it->second.T = T[i];
            i++;
            continue;
        }
        if((frame_it->first) > Headers[i])
        {
            i++;
        }
        Matrix3d R_inital = (Q[i].inverse()).toRotationMatrix();
        Vector3d P_inital = - R_inital * T[i];
        cv::eigen2cv(R_inital, tmp_r); //将Eigen变为OpenCV的cv::Mat类型。
        cv::Rodrigues(tmp_r, rvec);
        cv::eigen2cv(P_inital, t);

        frame_it->second.is_key_frame = false;
        vector<cv::Point3f> pts_3_vector;
        vector<cv::Point2f> pts_2_vector;
        for (auto &id_pts : frame_it->second.points)
        {
            int feature_id = id_pts.first;
            for (auto &i_p : id_pts.second)
            {
                it = sfm_tracked_points.find(feature_id);
                if(it != sfm_tracked_points.end())
                {
                    Vector3d world_pts = it->second;
                    cv::Point3f pts_3(world_pts(0), world_pts(1), world_pts(2));
                    pts_3_vector.push_back(pts_3);
                    Vector2d img_pts = i_p.second.head<2>();
                    cv::Point2f pts_2(img_pts(0), img_pts(1));
                    pts_2_vector.push_back(pts_2);
                }
            }
        }
        cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);     
        // cv::Mat K = (cv::Mat_<double>(3, 3) << 367.52449489660699555, 0, 368.07304293979439080, 0, 367.53022271275870025, 230.71178789661894371, 0, 0, 1);     
        // cv::Mat DD = (cv::Mat_<double>(4, 1) << -0.03029504397270243, 0.02456131832180794, -0.03688066102662716, 0.01626083592134776);     

        if(pts_3_vector.size() < 6)
        {
            cout << "pts_3_vector size " << pts_3_vector.size() << endl;
            DLOG(INFO) << "ot enough points for solve pnp!";
            return false;
        }

        //上边这段代码，主要就是在构造3D-2D的对应点，即pts_3_vector和pts_2_vector。
        /*3D-2D的PnP问题。
        pts_3_vector： w系下的被观测点。 
        pts_2_vector：在图像上的2D特征点。
        K是相机内参,形式如下：
            | fx  0  cx | 
            | 0  fy  cy | 
            | 0   0   1 | 
        D: 畸变系数矩阵，如果为Null/empty,按无畸变来处理。
        rvec和t：PnP输出的R，t [Input/Output]
        useExtrinsicGuess：即下边的1(true)，表示是否提供初始的猜想值，猜想值输入即前边的rvec, t
        ref: https://docs.opencv.org/3.4.8/d9/d0c/group__calib3d.html#ga549c2075fac14829ff4a58bc931c033d
        */
        //SONG: 求解PnP, 这里内参矩阵K用单位阵，是不是有问题?
        if (! cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t, 1))
        // if (! cv::solvePnP(pts_3_vector, pts_2_vector, K, DD, rvec, t, false, cv::SOLVEPNP_EPNP))
        {
            DLOG(INFO) << "solve pnp fail!";
            return false;
        }
        //paper[6] 崔华坤[29, 30]
        cv::Rodrigues(rvec, r);
        MatrixXd R_pnp,tmp_R_pnp;
        cv::cv2eigen(r, tmp_R_pnp);
        R_pnp = tmp_R_pnp.transpose();
        MatrixXd T_pnp;
        cv::cv2eigen(t, T_pnp);
        T_pnp = R_pnp * (-T_pnp);
        frame_it->second.R = R_pnp * RIC[0].transpose();
        frame_it->second.T = T_pnp;
    }
    if (visualInitialAlign())
        return true;
    else
    {
        LOG(INFO) << "misalign visual structure with IMU";
        return false;
    }

}

/*
视频和IMU的对齐。
    1. 估计gyro bias. solveGyroscopeBias
    2. 初始化速度、重力和尺度因子:对应代码中 LinearAlignment()
    3. 修正g: 对应代码中RefineGravity()
*/
bool Estimator::visualInitialAlign()
{
    TicToc t_g;
    VectorXd x;
    //solve scale
    bool result = VisualIMUAlignment(all_image_frame, Bgs, g, x);
    if(!result)
    {
        DLOG(INFO) << "solve g failed!";
        return false;
    }

    // change state
    for (int i = 0; i <= frame_count; i++)
    {
        Matrix3d Ri = all_image_frame[Headers[i]].R;
        Vector3d Pi = all_image_frame[Headers[i]].T;
        Ps[i] = Pi;
        Rs[i] = Ri;
        all_image_frame[Headers[i]].is_key_frame = true;
    }

    double s = (x.tail<1>())(0);
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);
    }
    for (int i = frame_count; i >= 0; i--)
        Ps[i] = s * Ps[i] - Rs[i] * TIC[0] - (s * Ps[0] - Rs[0] * TIC[0]);
    int kv = -1;
    map<double, ImageFrame>::iterator frame_i;
    for (frame_i = all_image_frame.begin(); frame_i != all_image_frame.end(); frame_i++)
    {
        if(frame_i->second.is_key_frame)
        {
            kv++;
            Vs[kv] = frame_i->second.R * x.segment<3>(kv * 3);
        }
    }

    Matrix3d R0 = Utility::g2R(g);
    double yaw = Utility::R2ypr(R0 * Rs[0]).x();
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    g = R0 * g; //这里需要debug一下，看看当camera倾斜时g的值，应该是(0,0,1g).
    //Matrix3d rot_diff = R0 * Rs[0].transpose();
    Matrix3d rot_diff = R0;
    for (int i = 0; i <= frame_count; i++)
    {
        Ps[i] = rot_diff * Ps[i];
        Rs[i] = rot_diff * Rs[i];
        Vs[i] = rot_diff * Vs[i];
    }
    DLOG(INFO) << "g0     " << g.transpose();
    DLOG(INFO) << "my R0  " << Utility::R2ypr(Rs[0]).transpose(); 

    f_manager.clearDepth();
    f_manager.triangulate(frame_count, Ps, Rs, tic, ric);

    return true;
}

//SONG:根据对极几何，先求F基础矩阵，再求R和T。
bool Estimator::relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l)
{
    // find previous frame which contians enough correspondance and parallex with newest frame
    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        vector<pair<Vector3d, Vector3d>> corres;
        corres = f_manager.getCorresponding(i, WINDOW_SIZE);//获取两个滑窗内所有对应（共视）的特征点在归一化平面上的坐标，并存入corres。
        //其实在MotionEstimator::solveRelativeRT内部也有对corres size的判断，需要大于15才执行PnP。
        if (corres.size() > 20)//在滑窗内寻找与当前帧的匹配特征点数较多的关键帧才能作为参考帧
        {
            double sum_parallax = 0;
            double average_parallax;
            for (int j = 0; j < int(corres.size()); j++)
            {
                Vector2d pts_0(corres[j].first(0), corres[j].first(1));
                Vector2d pts_1(corres[j].second(0), corres[j].second(1));
                double parallax = (pts_0 - pts_1).norm();
                sum_parallax = sum_parallax + parallax;
            }
            average_parallax = 1.0 * sum_parallax / int(corres.size());
            //平均视差要大于30
            //通过求基础矩阵 cv::findFundamentalMat 计算出当前帧到参考帧的 T
            //这里并不是拿所有滑窗内的特征点信息来计算RT，如果solveRelativeRT求得了一个有效的RT就直接return了。
            if(average_parallax * 460 > 30 && m_estimator.solveRelativeRT(corres, relative_R, relative_T))
            {
                l = i;
                DLOG(INFO) << "average_parallax " << average_parallax * 460
                << " choose l " << l
                << " and newest frame to triangulate the whole structure";
                return true;
            }
        }
    }
    return false;
}

void Estimator::vector2double()
{
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        para_Pose[i][0] = Ps[i].x();
        para_Pose[i][1] = Ps[i].y();
        para_Pose[i][2] = Ps[i].z();
        Quaterniond q{Rs[i]};
        para_Pose[i][3] = q.x();
        para_Pose[i][4] = q.y();
        para_Pose[i][5] = q.z();
        para_Pose[i][6] = q.w();

        if(USE_IMU)
        {
            para_SpeedBias[i][0] = Vs[i].x();
            para_SpeedBias[i][1] = Vs[i].y();
            para_SpeedBias[i][2] = Vs[i].z();

            para_SpeedBias[i][3] = Bas[i].x();
            para_SpeedBias[i][4] = Bas[i].y();
            para_SpeedBias[i][5] = Bas[i].z();

            para_SpeedBias[i][6] = Bgs[i].x();
            para_SpeedBias[i][7] = Bgs[i].y();
            para_SpeedBias[i][8] = Bgs[i].z();
        }
    }

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        para_Ex_Pose[i][0] = tic[i].x();
        para_Ex_Pose[i][1] = tic[i].y();
        para_Ex_Pose[i][2] = tic[i].z();
        Quaterniond q{ric[i]};
        para_Ex_Pose[i][3] = q.x();
        para_Ex_Pose[i][4] = q.y();
        para_Ex_Pose[i][5] = q.z();
        para_Ex_Pose[i][6] = q.w();
    }

    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        para_Feature[i][0] = dep(i);

    para_Td[0][0] = td;
}

void Estimator::double2vector()
{
    Vector3d origin_R0 = Utility::R2ypr(Rs[0]);
    Vector3d origin_P0 = Ps[0];

    if (failure_occur)
    {
        origin_R0 = Utility::R2ypr(last_R0);
        origin_P0 = last_P0;
        failure_occur = 0;
    }

    if(USE_IMU)
    {
        Vector3d origin_R00 = Utility::R2ypr(Quaterniond(para_Pose[0][6],
                                                          para_Pose[0][3],
                                                          para_Pose[0][4],
                                                          para_Pose[0][5]).toRotationMatrix());
        double y_diff = origin_R0.x() - origin_R00.x(); //SONG: yaw的差值
        //TODO
        Matrix3d rot_diff = Utility::ypr2R(Vector3d(y_diff, 0, 0));
        //SONG: 处理pitch在90度附近时的奇异值问题。但不知道为什么乘以T。
        if (abs(abs(origin_R0.y()) - 90) < 1.0 || abs(abs(origin_R00.y()) - 90) < 1.0)
        {
            DLOG(INFO) << "euler singular point!";
            rot_diff = Rs[0] * Quaterniond(para_Pose[0][6],
                                           para_Pose[0][3],
                                           para_Pose[0][4],
                                           para_Pose[0][5]).toRotationMatrix().transpose();
        }

        for (int i = 0; i <= WINDOW_SIZE; i++)
        {

            Rs[i] = rot_diff * Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();
            
            Ps[i] = rot_diff * Vector3d(para_Pose[i][0] - para_Pose[0][0],
                                    para_Pose[i][1] - para_Pose[0][1],
                                    para_Pose[i][2] - para_Pose[0][2]) + origin_P0;


                Vs[i] = rot_diff * Vector3d(para_SpeedBias[i][0],
                                            para_SpeedBias[i][1],
                                            para_SpeedBias[i][2]);

                Bas[i] = Vector3d(para_SpeedBias[i][3],
                                  para_SpeedBias[i][4],
                                  para_SpeedBias[i][5]);

                Bgs[i] = Vector3d(para_SpeedBias[i][6],
                                  para_SpeedBias[i][7],
                                  para_SpeedBias[i][8]);

        }
    }
    else
    {
        for (int i = 0; i <= WINDOW_SIZE; i++)
        {
            Rs[i] = Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();
            
            Ps[i] = Vector3d(para_Pose[i][0], para_Pose[i][1], para_Pose[i][2]);
        }
    }

    if(USE_IMU)
    {
        //SONG: tic和ric是经过精确测量并已作为已知量，此处被重新赋值是有问题的。同理下边的td.
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            tic[i] = Vector3d(para_Ex_Pose[i][0],
                              para_Ex_Pose[i][1],
                              para_Ex_Pose[i][2]);
            ric[i] = Quaterniond(para_Ex_Pose[i][6],
                                 para_Ex_Pose[i][3],
                                 para_Ex_Pose[i][4],
                                 para_Ex_Pose[i][5]).toRotationMatrix();
        }
    }

    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        dep(i) = para_Feature[i][0];
    f_manager.setDepth(dep);

    if(USE_IMU)
        td = para_Td[0][0];

}
/*
判断检测失败的条件：
1. 特征点数量太少;
2. 根据优化得到acc和gyro的bias太大;
3. 当前与上一帧位移差太大，特别是沿z轴方向的位移大于1米时;
4. 当前与上一帧转过的角度（绕欧拉轴的转角）太大。
*/
bool Estimator::failureDetection()
{
    return false;
    if (f_manager.last_track_num < 2)
    {
        LOG(INFO) << "little feature " << f_manager.last_track_num;
        //return true;
    }
    if (Bas[WINDOW_SIZE].norm() > 2.5)
    {
        LOG(INFO) << "big IMU acc bias estimation " << Bas[WINDOW_SIZE].norm();
        return true;
    }
    if (Bgs[WINDOW_SIZE].norm() > 1.0)
    {
        LOG(INFO) << "big IMU gyr bias estimation " << Bgs[WINDOW_SIZE].norm();
        return true;
    }
    /*
    if (tic(0) > 1)
    {
        LOG(INFO) << "big extri param estimation " << tic(0) > 1;
        return true;
    }
    */
    Vector3d tmp_P = Ps[WINDOW_SIZE];
    if ((tmp_P - last_P).norm() > 5)
    {
        // LOG(INFO) << "big translation";
        //return true;
    }
    if (abs(tmp_P.z() - last_P.z()) > 1)
    {
        // LOG(INFO) << "big z translation";
        //return true; 
    }
    Matrix3d tmp_R = Rs[WINDOW_SIZE];
    Matrix3d delta_R = tmp_R.transpose() * last_R;
    Quaterniond delta_Q(delta_R);
    double delta_angle;
    //根据四元数与欧拉轴角的转换关系: w=cos(a/2), 得绕欧拉轴的转角a=arccos(w)*2
    delta_angle = acos(delta_Q.w()) * 2.0 / 3.14 * 180.0;
    if (delta_angle > 50)
    {
        LOG(INFO) << "big delta_angle";
        //return true;
    }
    return false;
}

void Estimator::optimization()
{
    TicToc t_whole, t_prepare;
    vector2double();

    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    //loss_function = NULL;
    loss_function = new ceres::HuberLoss(1.0);
    //loss_function = new ceres::CauchyLoss(1.0 / FOCAL_LENGTH);
    //ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);
    for (int i = 0; i < frame_count + 1; i++)
    {//很奇怪，new了这么多，却没有delete，为什么不会内存泄漏？
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        //添加参数块，包括pose和q，共7个维度。
        problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);
        if(USE_IMU) //包括3维Vel, 3维accel和3维gyro的bias, 共9个维度。
            problem.AddParameterBlock(para_SpeedBias[i], SIZE_SPEEDBIAS);
    }
    if(!USE_IMU)//在优化过程中保持指定的参数块para_Pose[0]不变。
        problem.SetParameterBlockConstant(para_Pose[0]);

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        //将外参R,T 添加到参数块。
        problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_parameterization);
        //Vs[0].norm() 是求Vs[0]的范数，即sqrt(v_x*v_x + v_y*v_y + v_z*v_z),即判断合速度是否大于0.2 m/s
        if ((ESTIMATE_EXTRINSIC && frame_count == WINDOW_SIZE && Vs[0].norm() > 0.2) || openExEstimation)
        {
            // LOG(INFO) << "estimate extinsic param";
            openExEstimation = 1;
        }
        else
        { 
            //LOG(INFO) << "fix extinsic param";
            //如果不需要估计, 则para_Ex_Pose[i]不变。
            problem.SetParameterBlockConstant(para_Ex_Pose[i]); 
        }
    }
    //para_Td是啥？
    problem.AddParameterBlock(para_Td[0], 1);
    cout << "para_Td[0]:" << para_Td[0] << endl;
    if (!ESTIMATE_TD || Vs[0].norm() < 0.2)
        problem.SetParameterBlockConstant(para_Td[0]);

    if (last_marginalization_info && last_marginalization_info->valid)
    {
        // construct new marginlization_factor
        MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
        problem.AddResidualBlock(marginalization_factor, NULL,
                                 last_marginalization_parameter_blocks);
    }
    if(USE_IMU)
    {
        for (int i = 0; i < frame_count; i++)
        {
            int j = i + 1;
            if (pre_integrations[j]->sum_dt > 10.0)//如果imu数据跨度大于10s，则不参与优化。
                continue;
            IMUFactor* imu_factor = new IMUFactor(pre_integrations[j]);
            //添加IMU的Residual Block
            problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[j], para_SpeedBias[j]);
        }
    }

    int f_m_cnt = 0;
    int feature_index = -1;
    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < 4)
            continue;
 
        ++feature_index;

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
        
        Vector3d pts_i = it_per_id.feature_per_frame[0].point;

        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            if (imu_i != imu_j)
            {
                Vector3d pts_j = it_per_frame.point;
                ProjectionTwoFrameOneCamFactor *f_td = new ProjectionTwoFrameOneCamFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                 it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                problem.AddResidualBlock(f_td, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]);
            }

            if(STEREO && it_per_frame.is_stereo)
            {                
                Vector3d pts_j_right = it_per_frame.pointRight;
                if(imu_i != imu_j)
                {
                    ProjectionTwoFrameTwoCamFactor *f = new ProjectionTwoFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                 it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                    problem.AddResidualBlock(f, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]);
                }
                else
                {
                    ProjectionOneFrameTwoCamFactor *f = new ProjectionOneFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                 it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                    problem.AddResidualBlock(f, loss_function, para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]);
                }
               
            }
            f_m_cnt++;
        }
    }
    
    DLOG(INFO) << "visual measurement count: " << f_m_cnt;
    //printf("prepare for ceres: %f \n", t_prepare.toc());

    ceres::Solver::Options options;

    options.linear_solver_type = ceres::DENSE_SCHUR;
    //options.num_threads = 2;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = NUM_ITERATIONS;
    //options.use_explicit_schur_complement = true;
    //options.minimizer_progress_to_stdout = true;
    //options.use_nonmonotonic_steps = true;
    if (marginalization_flag == MARGIN_OLD)
        options.max_solver_time_in_seconds = SOLVER_TIME * 4.0 / 5.0;
    else
        options.max_solver_time_in_seconds = SOLVER_TIME;
    TicToc t_solver;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    //cout << summary.BriefReport() << endl;
    DLOG(INFO) << "Iterations : " << static_cast<int>(summary.iterations.size());
    // printf("optimization solver costs: %f \n", t_solver.toc());

    double2vector();
    //printf("frame_count: %d \n", frame_count);

    if(frame_count < WINDOW_SIZE)
        return;

    TicToc t_whole_marginalization;
    if (marginalization_flag == MARGIN_OLD)
    {
        MarginalizationInfo *marginalization_info = new MarginalizationInfo();
        vector2double();

        if (last_marginalization_info && last_marginalization_info->valid)
        {
            vector<int> drop_set;
            for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
            {
                if (last_marginalization_parameter_blocks[i] == para_Pose[0] ||
                    last_marginalization_parameter_blocks[i] == para_SpeedBias[0])
                    drop_set.push_back(i);
            }
            // construct new marginlization_factor
            MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                           last_marginalization_parameter_blocks,
                                                                           drop_set);
            marginalization_info->addResidualBlockInfo(residual_block_info);
        }

        if(USE_IMU)
        {
            if (pre_integrations[1]->sum_dt < 10.0)
            {
                IMUFactor* imu_factor = new IMUFactor(pre_integrations[1]);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(imu_factor, NULL,
                                                                           vector<double *>{para_Pose[0], para_SpeedBias[0], para_Pose[1], para_SpeedBias[1]},
                                                                           vector<int>{0, 1});
                marginalization_info->addResidualBlockInfo(residual_block_info);
            }
        }

        {
            int feature_index = -1;
            for (auto &it_per_id : f_manager.feature)
            {
                it_per_id.used_num = it_per_id.feature_per_frame.size();
                if (it_per_id.used_num < 4)
                    continue;

                ++feature_index;

                int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
                if (imu_i != 0)
                    continue;

                Vector3d pts_i = it_per_id.feature_per_frame[0].point;

                for (auto &it_per_frame : it_per_id.feature_per_frame)
                {
                    imu_j++;
                    if(imu_i != imu_j)
                    {
                        Vector3d pts_j = it_per_frame.point;
                        ProjectionTwoFrameOneCamFactor *f_td = new ProjectionTwoFrameOneCamFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                          it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f_td, loss_function,
                                                                                        vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]},
                                                                                        vector<int>{0, 3});
                        marginalization_info->addResidualBlockInfo(residual_block_info);
                    }
                    if(STEREO && it_per_frame.is_stereo)
                    {
                        Vector3d pts_j_right = it_per_frame.pointRight;
                        if(imu_i != imu_j)
                        {
                            ProjectionTwoFrameTwoCamFactor *f = new ProjectionTwoFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                          it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                           vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]},
                                                                                           vector<int>{0, 4});
                            marginalization_info->addResidualBlockInfo(residual_block_info);
                        }
                        else
                        {
                            ProjectionOneFrameTwoCamFactor *f = new ProjectionOneFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                          it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                           vector<double *>{para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]},
                                                                                           vector<int>{2});
                            marginalization_info->addResidualBlockInfo(residual_block_info);
                        }
                    }
                }
            }
        }

        TicToc t_pre_margin;
        marginalization_info->preMarginalize();
        DLOG(INFO) << "pre marginalization " << t_pre_margin.toc() << "ms";

        TicToc t_margin;
        marginalization_info->marginalize();
        DLOG(INFO) << "marginalization " << t_margin.toc() << " ms";

        std::unordered_map<long, double *> addr_shift;
        for (int i = 1; i <= WINDOW_SIZE; i++)
        {
            addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
            if(USE_IMU)
                addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
        }
        for (int i = 0; i < NUM_OF_CAM; i++)
            addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];

        addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];

        vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);

        if (last_marginalization_info)
            delete last_marginalization_info;
        last_marginalization_info = marginalization_info;
        last_marginalization_parameter_blocks = parameter_blocks;
        
    }
    else
    {
        if (last_marginalization_info &&
            std::count(std::begin(last_marginalization_parameter_blocks), std::end(last_marginalization_parameter_blocks), para_Pose[WINDOW_SIZE - 1]))
        {

            MarginalizationInfo *marginalization_info = new MarginalizationInfo();
            vector2double();
            if (last_marginalization_info && last_marginalization_info->valid)
            {
                vector<int> drop_set;
                for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
                {
                    LOG_IF(ERROR, !(last_marginalization_parameter_blocks[i] != para_SpeedBias[WINDOW_SIZE - 1])) 
                    << "last_marginalization_parameter_blocks[i] != para_SpeedBias[WINDOW_SIZE - 1]";

                    if (last_marginalization_parameter_blocks[i] == para_Pose[WINDOW_SIZE - 1])
                        drop_set.push_back(i);
                }
                // construct new marginlization_factor
                MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                               last_marginalization_parameter_blocks,
                                                                               drop_set);

                marginalization_info->addResidualBlockInfo(residual_block_info);
            }

            TicToc t_pre_margin;
            DLOG(INFO) << "begin marginalization";
            marginalization_info->preMarginalize();
            DLOG(INFO) << "end pre marginalization, " << t_pre_margin.toc() << " ms";

            TicToc t_margin;
            DLOG(INFO) << "begin marginalization";
            marginalization_info->marginalize();
            DLOG(INFO) << "end marginalization, " << t_margin.toc() << " ms";
            
            std::unordered_map<long, double *> addr_shift;
            for (int i = 0; i <= WINDOW_SIZE; i++)
            {
                if (i == WINDOW_SIZE - 1)
                    continue;
                else if (i == WINDOW_SIZE)
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
                    if(USE_IMU)
                        addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
                }
                else
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i];
                    if(USE_IMU)
                        addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i];
                }
            }
            for (int i = 0; i < NUM_OF_CAM; i++)
                addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];

            addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];

            
            vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);
            if (last_marginalization_info)
                delete last_marginalization_info;
            last_marginalization_info = marginalization_info;
            last_marginalization_parameter_blocks = parameter_blocks;
            
        }
    }
    //printf("whole marginalization costs: %f \n", t_whole_marginalization.toc());
    //printf("whole time for ceres: %f \n", t_whole.toc());
}

void Estimator::slideWindow()
{
    TicToc t_margin;
    if (marginalization_flag == MARGIN_OLD)
    {
        double t_0 = Headers[0];
        back_R0 = Rs[0];
        back_P0 = Ps[0];
        if (frame_count == WINDOW_SIZE)
        {
            for (int i = 0; i < WINDOW_SIZE; i++)
            {
                Headers[i] = Headers[i + 1];
                Rs[i].swap(Rs[i + 1]);
                Ps[i].swap(Ps[i + 1]);
                if(USE_IMU)
                {
                    std::swap(pre_integrations[i], pre_integrations[i + 1]);

                    dt_buf[i].swap(dt_buf[i + 1]);
                    linear_acceleration_buf[i].swap(linear_acceleration_buf[i + 1]);
                    angular_velocity_buf[i].swap(angular_velocity_buf[i + 1]);

                    Vs[i].swap(Vs[i + 1]);
                    Bas[i].swap(Bas[i + 1]);
                    Bgs[i].swap(Bgs[i + 1]);
                }
            }
            Headers[WINDOW_SIZE] = Headers[WINDOW_SIZE - 1];
            Ps[WINDOW_SIZE] = Ps[WINDOW_SIZE - 1];
            Rs[WINDOW_SIZE] = Rs[WINDOW_SIZE - 1];

            if(USE_IMU)
            {
                Vs[WINDOW_SIZE] = Vs[WINDOW_SIZE - 1];
                Bas[WINDOW_SIZE] = Bas[WINDOW_SIZE - 1];
                Bgs[WINDOW_SIZE] = Bgs[WINDOW_SIZE - 1];

                delete pre_integrations[WINDOW_SIZE];
                pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

                dt_buf[WINDOW_SIZE].clear();
                linear_acceleration_buf[WINDOW_SIZE].clear();
                angular_velocity_buf[WINDOW_SIZE].clear();
            }

            if (true || solver_flag == INITIAL)
            {
                map<double, ImageFrame>::iterator it_0;
                it_0 = all_image_frame.find(t_0);
                delete it_0->second.pre_integration;
                all_image_frame.erase(all_image_frame.begin(), it_0);
            }
            slideWindowOld();
        }
    }
    else
    {
        if (frame_count == WINDOW_SIZE)
        {
            Headers[frame_count - 1] = Headers[frame_count];
            Ps[frame_count - 1] = Ps[frame_count];
            Rs[frame_count - 1] = Rs[frame_count];

            if(USE_IMU)
            {
                for (unsigned int i = 0; i < dt_buf[frame_count].size(); i++)
                {
                    double tmp_dt = dt_buf[frame_count][i];
                    Vector3d tmp_linear_acceleration = linear_acceleration_buf[frame_count][i];
                    Vector3d tmp_angular_velocity = angular_velocity_buf[frame_count][i];
                    //processIMU不是已经做过预计分了吗，这里为什么又要重复做预计分？
                    pre_integrations[frame_count - 1]->push_back(tmp_dt, tmp_linear_acceleration, tmp_angular_velocity);

                    dt_buf[frame_count - 1].push_back(tmp_dt);
                    linear_acceleration_buf[frame_count - 1].push_back(tmp_linear_acceleration);
                    angular_velocity_buf[frame_count - 1].push_back(tmp_angular_velocity);
                }

                Vs[frame_count - 1] = Vs[frame_count];
                Bas[frame_count - 1] = Bas[frame_count];
                Bgs[frame_count - 1] = Bgs[frame_count];

                delete pre_integrations[WINDOW_SIZE];
                pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

                dt_buf[WINDOW_SIZE].clear();
                linear_acceleration_buf[WINDOW_SIZE].clear();
                angular_velocity_buf[WINDOW_SIZE].clear();
            }
            slideWindowNew();
        }
    }
}

void Estimator::slideWindowNew()
{
    sum_of_front++;
    f_manager.removeFront(frame_count);
}

void Estimator::slideWindowOld()
{
    sum_of_back++;

    bool shift_depth = solver_flag == NON_LINEAR ? true : false;
    if (shift_depth)
    {
        Matrix3d R0, R1;
        Vector3d P0, P1;
        R0 = back_R0 * ric[0];
        R1 = Rs[0] * ric[0];
        P0 = back_P0 + back_R0 * tic[0];
        P1 = Ps[0] + Rs[0] * tic[0];
        f_manager.removeBackShiftDepth(R0, P0, R1, P1);
    }
    else
        f_manager.removeBack();
}

//SONG:获取World frame下最新帧的pose.
void Estimator::getPoseInWorldFrame(Eigen::Matrix4d &T)
{
    T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = Rs[frame_count];
    T.block<3, 1>(0, 3) = Ps[frame_count];
}
//SONG:获取World frame下指定帧的pose.
void Estimator::getPoseInWorldFrame(int index, Eigen::Matrix4d &T)
{
    T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = Rs[index];
    T.block<3, 1>(0, 3) = Ps[index];
}
/*
根据上一帧和当前帧的位姿变化，预测下一阵的位姿，
然后将当前帧的特征点根据预测位姿变换后，得到预测的特征点。
坐标系依次变化:camera -> IMU -> world -> IMU -> camera
*/
void Estimator::predictPtsInNextFrame()
{
    //printf("predict pts in next frame\n");
    if(frame_count < 2)
        return;
    // predict next pose. Assume constant velocity motion
    Eigen::Matrix4d curT, prevT, nextT;
    getPoseInWorldFrame(curT); //得到在w系下，当最新状态的Rs和Ps
    getPoseInWorldFrame(frame_count - 1, prevT);//得到在w系下，当上一帧的Rs和Ps
    /*
        prevT: 上一帧的位姿
        curT: 当前帧位姿
        nextT: 待遇测的下一帧位姿。

        设prevT到curT的转换矩阵为 delta_T 满足：
        delta_T * prevT = curT
        则，delta_T = curT * (prevT.inverse)
        可预测下一状态位姿
        nextT = 则，delta_T * curT = curT * (prevT.inverse) * curT
    */
    nextT = curT * (prevT.inverse() * curT); 
    map<int, Eigen::Vector3d> predictPts;

    for (auto &it_per_id : f_manager.feature)
    {
        if(it_per_id.estimated_depth > 0)
        {
            int firstIndex = it_per_id.start_frame;
            int lastIndex = it_per_id.start_frame + it_per_id.feature_per_frame.size() - 1;
            //printf("cur frame index  %d last frame index %d\n", frame_count, lastIndex);
            //如果特征点跟踪数大于2，且同时在最后一个滑窗出现，即被当前帧检测到。
            if((int)it_per_id.feature_per_frame.size() >= 2 && lastIndex == frame_count)
            {
                // 将归一化平面的3D坐标(x/z,y/z,1), 还原到原始3D位置，即(x,y,z). 此处的z即depth。
                double depth = it_per_id.estimated_depth;
                //将pt从camera转到imu坐标系。
                Vector3d pts_j = ric[0] * (depth * it_per_id.feature_per_frame[0].point) + tic[0];
                //将pts_j转到世界坐标系，即w系。
                Vector3d pts_w = Rs[firstIndex] * pts_j + Ps[firstIndex];
                //得到在body(imu)系下的位置，没理解为什么是 (pts_w - nextT_Ps)
                Vector3d pts_local = nextT.block<3, 3>(0, 0).transpose() * (pts_w - nextT.block<3, 1>(0, 3));
                //pts_cam是将pts_local从body(imu)系下转到camera系下。
                Vector3d pts_cam = ric[0].transpose() * (pts_local - tic[0]);
                int ptsIndex = it_per_id.feature_id;
                predictPts[ptsIndex] = pts_cam;
            }
        }
    }
    featureTracker.setPrediction(predictPts);//预测下一帧特征点的坐标。
    //printf("estimator output %d predict pts\n",(int)predictPts.size());
}
//SONG:计算重映射误差。
double Estimator::reprojectionError(Matrix3d &Ri, Vector3d &Pi, Matrix3d &rici, Vector3d &tici,
                                 Matrix3d &Rj, Vector3d &Pj, Matrix3d &ricj, Vector3d &ticj, 
                                 double depth, Vector3d &uvi, Vector3d &uvj)
{
    Vector3d pts_w = Ri * (rici * (depth * uvi) + tici) + Pi;
    Vector3d pts_cj = ricj.transpose() * (Rj.transpose() * (pts_w - Pj) - ticj);
    Vector2d residual = (pts_cj / pts_cj.z()).head<2>() - uvj.head<2>();
    double rx = residual.x();
    double ry = residual.y();
    return sqrt(rx * rx + ry * ry);
}

void Estimator::outliersRejection(set<int> &removeIndex)
{
    //return;
    int feature_index = -1;
    for (auto &it_per_id : f_manager.feature)
    {
        double err = 0;
        int errCnt = 0;
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < 4)
            continue;
        feature_index ++;
        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
        Vector3d pts_i = it_per_id.feature_per_frame[0].point;
        double depth = it_per_id.estimated_depth;
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            if (imu_i != imu_j)
            {
                Vector3d pts_j = it_per_frame.point;             
                double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0], 
                                                    Rs[imu_j], Ps[imu_j], ric[0], tic[0],
                                                    depth, pts_i, pts_j);
                err += tmp_error;
                errCnt++;
                //printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
            }
            // need to rewrite projecton factor.........
            if(STEREO && it_per_frame.is_stereo)
            {
                
                Vector3d pts_j_right = it_per_frame.pointRight;
                if(imu_i != imu_j)
                {            
                    double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0], 
                                                        Rs[imu_j], Ps[imu_j], ric[1], tic[1],
                                                        depth, pts_i, pts_j_right);
                    err += tmp_error;
                    errCnt++;
                    //printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
                }
                else
                {
                    double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0], 
                                                        Rs[imu_j], Ps[imu_j], ric[1], tic[1],
                                                        depth, pts_i, pts_j_right);
                    err += tmp_error;
                    errCnt++;
                    //printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
                }       
            }
        }
        double ave_err = err / errCnt;
        if(ave_err * FOCAL_LENGTH > 3)
            removeIndex.insert(it_per_id.feature_id);

    }
}

//SONG:IMU快速计分求当前的速度和位置。
Eigen::Vector3d Estimator::fastPredictIMU(double t, Eigen::Vector3d linear_acceleration, Eigen::Vector3d angular_velocity)
{
    double dt = t - latest_time; //SONG:Delta t
    latest_time = t;
    Eigen::Vector3d un_acc_0 = latest_Q * (latest_acc_0 - latest_Ba) - g; //求线性加速度。//正常来讲，一个四元数是不能和一个向量直接做乘法的，维度都对不上，参看EigenTest1
    Eigen::Vector3d un_gyr = 0.5 * (latest_gyr_0 + angular_velocity) - latest_Bg;//取相邻两个gyro的均值。
    latest_Q = latest_Q * Utility::deltaQ(un_gyr * dt); //由gyro更新姿态矩阵
    Eigen::Vector3d un_acc_1 = latest_Q * (linear_acceleration - latest_Ba) - g; //求线性加速度
    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);//求相邻两个线性加速度的均值。
    latest_P = latest_P + dt * latest_V + 0.5 * dt * dt * un_acc; //推算位置和速度。
    latest_V = latest_V + dt * un_acc;
    latest_acc_0 = linear_acceleration;
    latest_gyr_0 = angular_velocity;
    return un_acc;
}

void Estimator::updateLatestStates()
{
    mPropagate.lock();
    latest_time = Headers[frame_count] + td;
    latest_P = Ps[frame_count];
    latest_Q = Rs[frame_count];
    latest_V = Vs[frame_count];
    latest_Ba = Bas[frame_count];
    latest_Bg = Bgs[frame_count];
    latest_acc_0 = acc_0;
    latest_gyr_0 = gyr_0;
    mBuf.lock();
    queue<pair<double, Eigen::Vector3d>> tmp_accBuf = accBuf;
    queue<pair<double, Eigen::Vector3d>> tmp_gyrBuf = gyrBuf;
    mBuf.unlock();
    while(!tmp_accBuf.empty())
    {
        double t = tmp_accBuf.front().first;
        Eigen::Vector3d acc = tmp_accBuf.front().second;
        Eigen::Vector3d gyr = tmp_gyrBuf.front().second;
        fastPredictIMU(t, acc, gyr);
        tmp_accBuf.pop();
        tmp_gyrBuf.pop();
    }
    mPropagate.unlock();

    // cout << "latest_time:" << fixed << setprecision(3) << latest_time << "(s)" << endl;
    // cout << latest_Ba.transpose() << "," << latest_Bg.transpose() << endl;
    saveIMUBias(latest_time, latest_Ba, latest_Bg, latest_P, latest_V);
}

//SONG: 保存accel和gyro的bias到文件.
//time IS header.stamp.toSec();
void Estimator::saveIMUBias(double time, const Eigen::Vector3d accel_bias, const Eigen::Vector3d gyro_bias,
                            const Eigen::Vector3d position, const Eigen::Vector3d velocity)
{
    const float R2D = 180/M_PI;
    // m_fout_imu_bias.precision(0);
    m_fout_imu_bias << time /** 1e9 */ << ",";
    // m_fout_imu_bias.precision(5);
    m_fout_imu_bias << accel_bias.x() << ","
                  << accel_bias.y() << ","
                  << accel_bias.z() << ","
                  << gyro_bias.x() * R2D << ","
                  << gyro_bias.y() * R2D << ","
                  << gyro_bias.z() * R2D << ","
                  << position.x()  << ","
                  << position.y()  << ","
                  << position.z()  << ","
                  << velocity.x()  << ","
                  << velocity.y()  << ","
                  << velocity.z()  << endl;
}

void Estimator::saveLinearAcc(double time, const Eigen::Vector3d accel)
{
    m_fout_linear_acc << time << "," << accel.x() << "," << accel.y() << "," << accel.z() << endl;    
}

std::tm* Estimator::getCurTime()
{
    std::chrono::time_point<std::chrono::system_clock,std::chrono::milliseconds> tp = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
    auto tmp=std::chrono::duration_cast<std::chrono::milliseconds>(tp.time_since_epoch());
    std::time_t timestamp = tmp.count();
	
    int64 milli = timestamp + (int64)8*60*60*1000;
    auto mTime = std::chrono::milliseconds(milli);
    auto tp_=std::chrono::time_point<std::chrono::system_clock,std::chrono::milliseconds>(mTime);
    auto tt = std::chrono::system_clock::to_time_t(tp_);
    std::tm* now = std::gmtime(&tt);
    return now;
}

bool Estimator::CreateLogFiles()
{
    //record accel and gyro bias evaluated by VINS.
    std::tm* now = getCurTime();
    char time_stamp_ch[32] = {0};
    sprintf(time_stamp_ch,"%4d%02d%02d_%02d%02d%02d",now->tm_year+1900, now->tm_mon+1,now->tm_mday,now->tm_hour,now->tm_min,now->tm_sec);

    std::string file_name(time_stamp_ch, time_stamp_ch + strlen(time_stamp_ch));
    file_name = OUTPUT_FOLDER + "/" + "imu_bias_" + file_name + ".csv";
    m_fout_imu_bias = ofstream(file_name, ios::app);
    m_fout_imu_bias.setf(ios::fixed, ios::floatfield); // ios_base::fixed:设置cout为定点输出格式; ios_base::floatfield:设置输出时按浮点格式，小数点后有6位数字
    if (!m_fout_imu_bias)
    {
        LOG(ERROR) << "open file " << file_name << " failed!";
        return false;
    }

    file_name.clear();
    file_name.assign(time_stamp_ch, time_stamp_ch + strlen(time_stamp_ch));
    file_name = OUTPUT_FOLDER + "/" + "linear_acc_" + file_name + ".csv";
    m_fout_linear_acc = ofstream(file_name, ios::app);
    m_fout_linear_acc.setf(ios::fixed, ios::floatfield);
    if (!m_fout_linear_acc)
    {
        LOG(ERROR) << "open file " << file_name << " failed!";
        return false;
    }

    return true;
}