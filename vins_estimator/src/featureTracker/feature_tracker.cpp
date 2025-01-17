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

#include "feature_tracker.h"

//判断像素坐标是否在图像边界范围内。
bool FeatureTracker::inBorder(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < col - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < row - BORDER_SIZE;
}

//SONG: 计算两个坐标点的欧氏距离。
double distance(cv::Point2f pt1, cv::Point2f pt2)
{
    //printf("pt1: %f %f pt2: %f %f\n", pt1.x, pt1.y, pt2.x, pt2.y);
    double dx = pt1.x - pt2.x;
    double dy = pt1.y - pt2.y;
    return sqrt(dx * dx + dy * dy);
}

//SONG: status中的值为0时，删除v中的对应项，否则保留。
//下边的两个reduceVector，可以用利用函数重载合并为一个函数。
void reduceVector(vector<cv::Point2f> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

void reduceVector(vector<int> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

FeatureTracker::FeatureTracker()
{
    stereo_cam = 0;
    n_id = 0;
    hasPrediction = false;
}

//SONG:以特征点为圆心，MIN_DIST(默认30个像素)为半径，在mask上做圆，圆内部用0填充。
// 如果两个特征点挨得很近小于MIN_DIST的距离，则只按照第一个特征点为圆心画圆，第二个特征点对应的圆就忽略不画了。
void FeatureTracker::setMask()
{
    //构造一个图像。
    mask = cv::Mat(row, col, CV_8UC1, cv::Scalar(255));

    // prefer to keep features that are tracked for long time
    //cnt_pts_id 是一个容器，保存了当前帧所有特征点的信息，包括每个特征点的坐标，id以及连续跟踪了多少帧。
    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;
    //       跟踪计数器     特征点坐标   特征点id
    //cnt_pts_id的第一个key是该特征点的跟踪计数器，第二个key是特征点坐标，value是特征点的id    
    for (unsigned int i = 0; i < cur_pts.size(); i++)
        cnt_pts_id.push_back(make_pair(track_cnt[i], make_pair(cur_pts[i], ids[i])));

    //cnt_pts_id按照track_cnt中记录的连续跟踪数量的计数器从大到小排序。选出跟踪最稳定的特征点。
    sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b)
         {
            return a.first > b.first;
         });

    cur_pts.clear();
    ids.clear();
    track_cnt.clear();

    //cur_pts,ids,track_cnt都按照cnt_pts_id记录的特征点的顺序（按照跟踪计数，从大到小排序，即按照跟踪质量排序）
    for (auto &it : cnt_pts_id)
    {
        if (mask.at<uchar>(it.second.first) == 255)//如果之前已有特征点对应的圆将本像素设置为0，则不再重复画圆了。
        {
            cur_pts.push_back(it.second.first);
            ids.push_back(it.second.second);
            track_cnt.push_back(it.first);
            cv::circle(mask, it.second.first, MIN_DIST, 0, -1); //最后的-1表示是实心圆，即内部也被填充。
        }
    }
}

//SONG:计算两点间的欧氏距离
double FeatureTracker::distance(cv::Point2f &pt1, cv::Point2f &pt2)
{
    //printf("pt1: %f %f pt2: %f %f\n", pt1.x, pt1.y, pt2.x, pt2.y);
    double dx = pt1.x - pt2.x;
    double dy = pt1.y - pt2.y;
    return sqrt(dx * dx + dy * dy);
}

/*
SONG: map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame
可参考 FeaturePerFrame 的定义.
第一个int为feature的id，vector里面的int为相机id（0为左边的camera0，1为右边的camera1），
后面的Eigen::Matrix类型里面包含该特征点在该相机下的信息，
分别为:归一化平面坐标（x,y,z=1），像素坐标（u,v），像素移动速度（v_x,v_y），共七维。
将featureFrame加入到featurebuf中，传到后端进行图像处理。
*/
map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> FeatureTracker::trackImage(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1)
{
    TicToc t_r;
    cur_time = _cur_time;
    cur_img = _img;
    row = cur_img.rows;
    col = cur_img.cols;
    cv::Mat rightImg = _img1; //双目有效
    /*
    {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        clahe->apply(cur_img, cur_img);
        if(!rightImg.empty())
            clahe->apply(rightImg, rightImg);
    }
    */
    cur_pts.clear(); //cur_pts 保存的是当前帧的特征点，先清空。

    //下边这个if块的功能是：通过LK光流算法，在当前帧图像中检测匹配的跟踪点，并通过一些策略剔除误匹配的跟踪点。
    if (prev_pts.size() > 0) //如果上一帧检测有特征点
    {
        TicToc t_o;
        vector<uchar> status;
        vector<float> err;
        if(hasPrediction) //如果之前有做预测，则先按预测的特征点进行匹配。
        {
            cur_pts = predict_pts;
            //每级金字塔的搜索窗口大小为Size(21, 21)，当摄像头移动或转动过快，
            //或图像分辨率较高时可能会找不到匹配的特征点，这样会有问题。

            //cur_pts即是输入又是输出，保存匹配成功的特征点。
            //status是标志位，如果找到了对应特征的流，则将向量的对应元素设置为1；否则，置0。
            //参考calcOpticalFlowPyrLK的参数说明。[https://blog.csdn.net/liangchunjiang/article/details/79869830]
            cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 1, 
            cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
            
            int succ_num = 0; //成功跟踪上的特征点的数量
            for (size_t i = 0; i < status.size(); i++)
            {
                if (status[i])
                    succ_num++;
            }
            if (succ_num < 10) //如果匹配的特征点数量太少，则增加金字塔层级，3级。
               cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 3);
        }
        else //如果没有上一帧对特征点的预测值，则用3级金字塔去做特征点跟踪。
            cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 3);
        // reverse check  反向检查
        if(FLOW_BACK) //SONG:由yaml配置，用于剔除干扰特征点。perform forward and backward optical flow to improve feature tracking accuracy
        {
            vector<uchar> reverse_status;
            vector<cv::Point2f> reverse_pts = prev_pts;
            //交换当前帧和上一帧特征点的位置，即由当前帧特征点作为输入，从上一帧特征点中找匹配。思路很好。
            cv::calcOpticalFlowPyrLK(cur_img, prev_img, cur_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 1, 
            cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
            //cv::calcOpticalFlowPyrLK(cur_img, prev_img, cur_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 3); 
            for(size_t i = 0; i < status.size(); i++)
            {           //SONG:根据“forward and backward optical flow”， 若欧氏距离>0.5个像素，也应该被剔除。
                if(status[i] && reverse_status[i] && distance(prev_pts[i], reverse_pts[i]) <= 0.5)
                {
                    status[i] = 1;
                }
                else
                    status[i] = 0;
            }
        }
        
        for (int i = 0; i < int(cur_pts.size()); i++)
            if (status[i] && !inBorder(cur_pts[i])) //防越界处理。
                status[i] = 0;
        //根据status剔除误匹配的特征点。
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        DLOG(INFO) << "temporal optical flow costs: " << t_o.toc() << "ms"; //看一下光流跟踪这部分处理的耗时。
        //printf("track cnt %d\n", (int)ids.size());
    }
    //每个特征点都有对应的id，track_cnt记录了每个特征点被跟踪到的计数器。
    for (auto &n : track_cnt) //特征点跟踪计数累加
        n++;

    //下边这个if块的功能是：如果当前特征点的数量未达到MAX_CNT，则通过goodFeaturesToTrack来获取新的角点补充到跟踪点中。
    if (1)
    {
        //rejectWithF();
        DLOG(INFO) << "set mask begins";
        TicToc t_m;
        setMask(); //设置mask：以特征点为圆心，MIN_DIST为半径，在mask上做内部为0像素值的圆。
        DLOG(INFO) << "set mask costs " << t_m.toc() << " ms";
        DLOG(INFO) << "detect feature begins";

        TicToc t_t;
        int n_max_cnt = MAX_CNT - static_cast<int>(cur_pts.size());
        if (n_max_cnt > 0) //如果跟踪点数量小于阈值MAX_CNT
        {
            if(mask.empty())
                cout << "mask is empty " << endl;
            if (mask.type() != CV_8UC1)
                cout << "mask type wrong " << endl;
            //n_pts为输出角点vector；
            //MAX_CNT - cur_pts.size():最大角点数目
            //如果在其周围MIN_DIST范围内存在其他更强角点，则将此角点删除，即最大值抑制。
            //goodFeaturesToTrack默认检测shi-tomasi角点
            //mask的作用是指定ROI，即检测角点时把已有的跟踪点遮住。
            cv::goodFeaturesToTrack(cur_img, n_pts, MAX_CNT - cur_pts.size(), 0.01, MIN_DIST, mask);
        }
        else
            n_pts.clear();
        DLOG(INFO) << "sdetect feature costs: " << t_t.toc() << " ms";

        //SONG:将新的角点补充到特征点中。
        for (auto &p : n_pts)
        {
            cur_pts.push_back(p);
            ids.push_back(n_id++); //为新的特征点分配一个id。
            track_cnt.push_back(1);
        }
        //printf("feature cnt after add %d\n", (int)ids.size());
    }

    //cur_un_pts:将当前帧的特征点cur_pts去畸变后，再转到归一化平面上的特征点。这里m_camera用于指定相机模型来做畸变矫正。
    cur_un_pts = undistortedPts(cur_pts, m_camera[0]);
    pts_velocity = ptsVelocity(ids, cur_un_pts, cur_un_pts_map, prev_un_pts_map);

    if(!_img1.empty() && stereo_cam) //对双目相机的处理。是在左右目图像帧中找光流点的匹配，而不是在前后帧。
    {
        ids_right.clear();
        cur_right_pts.clear();
        cur_un_right_pts.clear();
        right_pts_velocity.clear();
        cur_un_right_pts_map.clear();
        if(!cur_pts.empty())
        {
            //printf("stereo image; track feature on right image\n");
            vector<cv::Point2f> reverseLeftPts;
            vector<uchar> status, statusRightLeft;
            vector<float> err;
            // 使用LK光流算法，在当前帧的左右目图像中找对应的特征点。
            // cur left ---- cur right
            cv::calcOpticalFlowPyrLK(cur_img, rightImg, cur_pts, cur_right_pts, status, err, cv::Size(21, 21), 3);
            // reverse check cur right ---- cur left
            if(FLOW_BACK) //剔除部分匹配不太好的特征点。
            {
                cv::calcOpticalFlowPyrLK(rightImg, cur_img, cur_right_pts, reverseLeftPts, statusRightLeft, err, cv::Size(21, 21), 3);
                for(size_t i = 0; i < status.size(); i++)
                {
                    if(status[i] && statusRightLeft[i] && inBorder(cur_right_pts[i]) && distance(cur_pts[i], reverseLeftPts[i]) <= 0.5)
                        status[i] = 1;
                    else
                        status[i] = 0;
                }
            }

            ids_right = ids;
            reduceVector(cur_right_pts, status);
            reduceVector(ids_right, status);
            // only keep left-right pts
            /*
            reduceVector(cur_pts, status);
            reduceVector(ids, status);
            reduceVector(track_cnt, status);
            reduceVector(cur_un_pts, status);
            reduceVector(pts_velocity, status);
            */
            cur_un_right_pts = undistortedPts(cur_right_pts, m_camera[1]);
            right_pts_velocity = ptsVelocity(ids_right, cur_un_right_pts, cur_un_right_pts_map, prev_un_right_pts_map);
        }
        prev_un_right_pts_map = cur_un_right_pts_map;
    }
    if(SHOW_TRACK) //SONG:如果需要publish tracking image 到 topic。
        drawTrack(cur_img, rightImg, ids, cur_pts, cur_right_pts, prevLeftPtsMap);

    prev_img = cur_img;
    prev_pts = cur_pts;
    prev_un_pts = cur_un_pts;
    prev_un_pts_map = cur_un_pts_map;
    prev_time = cur_time;
    hasPrediction = false;

    prevLeftPtsMap.clear();
    for(size_t i = 0; i < cur_pts.size(); i++)
        prevLeftPtsMap[ids[i]] = cur_pts[i];

    //下边将7维的特征信息放到featureFrame中返回。
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
    for (size_t i = 0; i < ids.size(); i++)
    {
        int feature_id = ids[i];
        double x, y ,z;
        x = cur_un_pts[i].x;
        y = cur_un_pts[i].y;
        z = 1;
        double p_u, p_v;
        p_u = cur_pts[i].x;
        p_v = cur_pts[i].y;
        int camera_id = 0; //camera_id: 0 左目  camera_id:1 右目
        double velocity_x, velocity_y;
        velocity_x = pts_velocity[i].x;
        velocity_y = pts_velocity[i].y;

        Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
        xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
        featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
    }

    if (!_img1.empty() && stereo_cam)
    {
        for (size_t i = 0; i < ids_right.size(); i++)
        {
            int feature_id = ids_right[i];
            double x, y ,z;
            x = cur_un_right_pts[i].x;
            y = cur_un_right_pts[i].y;
            z = 1;
            double p_u, p_v;
            p_u = cur_right_pts[i].x;
            p_v = cur_right_pts[i].y;
            int camera_id = 1;
            double velocity_x, velocity_y;
            velocity_x = right_pts_velocity[i].x;
            velocity_y = right_pts_velocity[i].y;

            Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
            xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
            featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
        }
    }

    //printf("feature track whole time %f\n", t_r.toc());
    return featureFrame;
}

//SONG:利用求基础矩阵F来剔除被舍弃的特征点。
void FeatureTracker::rejectWithF()
{
    if (cur_pts.size() >= 8)
    {
        DLOG(INFO) << "FM ransac begins";
        TicToc t_f;
        //SONG:un_cur_pts和un_prev_pts是像素平面的像素点在投影线上距离FOCAL_LENGTH处的点。相机模型由m_camera确定，比如pinhole、mei或kannala_brandt等。
        // 为什么不使用cur_pts和prev_pts，而是使用un_cur_pts和un_prev_pts来求基础矩阵F呢？难道cur_pts和prev_pts乘以FOCAL_LENGTH后求F，精度会更高？
        vector<cv::Point2f> un_cur_pts(cur_pts.size()), un_prev_pts(prev_pts.size());
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            Eigen::Vector3d tmp_p;
            m_camera[0]->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + col / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + row / 2.0;
            un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

            m_camera[0]->liftProjective(Eigen::Vector2d(prev_pts[i].x, prev_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + col / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + row / 2.0;
            un_prev_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
        }

        vector<uchar> status;
        //SONG: 此处的findFundamentalMat并不是为了求基础矩阵F,而只是求status;
        // status:具有N个元素的输出数组，在计算过程中没有被舍弃的点，元素被被置为1；否则置为0。
        // 这个数组只可以在方法RANSAC and LMedS 情况下使用；在其它方法的情况下，status一律被置为1。这个参数是可选参数。
        cv::findFundamentalMat(un_cur_pts, un_prev_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
        int size_a = cur_pts.size();
        // 根据status的标志位，清理prev_pts内被舍弃的特征点。
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(cur_un_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        //打印一下特征点数量的变化情况。
        DLOG(INFO) << "FM ransac: " << size_a << " -> " << cur_pts.size() << ": " << 1.0 * cur_pts.size() / size_a;
        DLOG(INFO) << "FM ransac costs: " <<  t_f.toc() << "ms" ;
    }
}

//SONG:读取摄像机的内参和pinhole/mei/equ模型的校准参数。
void FeatureTracker::readIntrinsicParameter(const vector<string> &calib_file)
{
    for (size_t i = 0; i < calib_file.size(); i++)
    {
        DLOG(INFO) << "reading paramerter of camera " << calib_file[i].c_str();
        camodocal::CameraPtr camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file[i]);
        m_camera.push_back(camera);
    }
    if (calib_file.size() == 2)
        stereo_cam = 1;
}

/*
SONG:这个函数目前没有被调用.
功能是:根据畸变参数,将畸变图像转为非畸变图像.
下边cv::Mat.at(idx)取像素点的方式效率非常低。
*/
void FeatureTracker::showUndistortion(const string &name)
{
    cv::Mat undistortedImg(row + 600, col + 600, CV_8UC1, cv::Scalar(0)); //校准后未畸变的图像尺寸会变大,所以往外扩600像素,其实600不一定够,需要看原始图像尺寸和畸变程度.
    vector<Eigen::Vector2d> distortedp, undistortedp;
    for (int i = 0; i < col; i++)
        for (int j = 0; j < row; j++)
        {
            Eigen::Vector2d a(i, j);
            Eigen::Vector3d b;
            m_camera[0]->liftProjective(a, b);
            distortedp.push_back(a);
            undistortedp.push_back(Eigen::Vector2d(b.x() / b.z(), b.y() / b.z()));
            //printf("%f,%f->%f,%f,%f\n)\n", a.x(), a.y(), b.x(), b.y(), b.z());
        }
    for (int i = 0; i < int(undistortedp.size()); i++)
    {
        cv::Mat pp(3, 1, CV_32FC1);
        pp.at<float>(0, 0) = undistortedp[i].x() * FOCAL_LENGTH + col / 2;
        pp.at<float>(1, 0) = undistortedp[i].y() * FOCAL_LENGTH + row / 2;
        pp.at<float>(2, 0) = 1.0;
        //cout << trackerData[0].K << endl;
        //printf("%lf %lf\n", p.at<float>(1, 0), p.at<float>(0, 0));
        //printf("%lf %lf\n", pp.at<float>(1, 0), pp.at<float>(0, 0));
        if (pp.at<float>(1, 0) + 300 >= 0 && pp.at<float>(1, 0) + 300 < row + 600 && pp.at<float>(0, 0) + 300 >= 0 && pp.at<float>(0, 0) + 300 < col + 600)
        {
            undistortedImg.at<uchar>(pp.at<float>(1, 0) + 300, pp.at<float>(0, 0) + 300) = cur_img.at<uchar>(distortedp[i].y(), distortedp[i].x());
        }
        else
        {
            // DLOG(INFO) << "(" << distortedp[i].y << " " << distortedp[i].x << ") -> (" << pp.at<float>(1, 0) << " " << pp.at<float>(0, 0) << ")";
        }
    }
    // turn the following code on if you need
    // cv::imshow(name, undistortedImg);
    // cv::waitKey(0);
}

//SONG:将一系列的像素坐标pts去畸变，并转为归一化平面坐标un_pts
vector<cv::Point2f> FeatureTracker::undistortedPts(vector<cv::Point2f> &pts, camodocal::CameraPtr cam)
{
    vector<cv::Point2f> un_pts;
    for (unsigned int i = 0; i < pts.size(); i++)
    {
        Eigen::Vector2d a(pts[i].x, pts[i].y);
        Eigen::Vector3d b;
        cam->liftProjective(a, b);
        un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
    }
    return un_pts;
}

//SONG:由相邻帧的特征点也就是cur_id_pts和prev_id_pts对应的特征点，计算并返回特征点的相对速度。
vector<cv::Point2f> FeatureTracker::ptsVelocity(vector<int> &ids, vector<cv::Point2f> &pts, 
                                            map<int, cv::Point2f> &cur_id_pts, map<int, cv::Point2f> &prev_id_pts)
{
    vector<cv::Point2f> pts_velocity;
    cur_id_pts.clear();
    //通过pts来构造 cur_id_pts
    for (unsigned int i = 0; i < ids.size(); i++)
    {
        cur_id_pts.insert(make_pair(ids[i], pts[i]));
    }
    /*
        如果prev_id_pts不为空：则将速度放入pts_velocity
        否则：用[0,0]填充pts_velocity
    */
    // caculate points velocity
    if (!prev_id_pts.empty())
    {
        double dt = cur_time - prev_time;
        
        for (unsigned int i = 0; i < pts.size(); i++)
        {
            std::map<int, cv::Point2f>::iterator it;
            it = prev_id_pts.find(ids[i]); //在上一帧的[id,特征点]map中找指定的id。
            if (it != prev_id_pts.end())//如果找到指定id,则计算x和y方向的速度，并放入pts_velocity
            {
                double v_x = (pts[i].x - it->second.x) / dt;
                double v_y = (pts[i].y - it->second.y) / dt;
                pts_velocity.push_back(cv::Point2f(v_x, v_y));
            }
            else
                pts_velocity.push_back(cv::Point2f(0, 0));

        }
    }
    else
    {
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            pts_velocity.push_back(cv::Point2f(0, 0));
        }
    }
    return pts_velocity;
}

/*
SONG:
drawTrack : 绘制跟踪点
1. 如果是双目相机，则合并两幅图
2. 左半图：
    2.1 如果某个特征点连续被跟踪了很久（超过20帧），则显示红色，否则显示蓝色
    2.2 显示光流的箭头
3. 右半图：
    3.1 特征点显示为绿色
*/
void FeatureTracker::drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight, 
                               vector<int> &curLeftIds,
                               vector<cv::Point2f> &curLeftPts, 
                               vector<cv::Point2f> &curRightPts,
                               map<int, cv::Point2f> &prevLeftPtsMap)
{
    //int rows = imLeft.rows;
    int cols = imLeft.cols;
    if (!imRight.empty() && stereo_cam) //SONG:如果右侧图片不为空，且是stereo camera，则把左右两个图拼接起来,拼接后的图为imTrack。 
        cv::hconcat(imLeft, imRight, imTrack);
    else
        imTrack = imLeft.clone();       //SONG:如果是monocular camera,则取左侧图为imTrack。
    cv::cvtColor(imTrack, imTrack, CV_GRAY2RGB);

    for (size_t j = 0; j < curLeftPts.size(); j++)
    {
        double len = std::min(1.0, 1.0 * track_cnt[j] / 20);
        cv::circle(imTrack, curLeftPts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
    }
    if (!imRight.empty() && stereo_cam)
    {
        for (size_t i = 0; i < curRightPts.size(); i++)
        {
            cv::Point2f rightPt = curRightPts[i];
            rightPt.x += cols;
            cv::circle(imTrack, rightPt, 2, cv::Scalar(0, 255, 0), 2);
            //cv::Point2f leftPt = curLeftPtsTrackRight[i];
            //cv::line(imTrack, leftPt, rightPt, cv::Scalar(0, 255, 0), 1, 8, 0);
        }
    }
    
    map<int, cv::Point2f>::iterator mapIt;
    for (size_t i = 0; i < curLeftIds.size(); i++)
    {
        int id = curLeftIds[i];
        mapIt = prevLeftPtsMap.find(id);
        if(mapIt != prevLeftPtsMap.end())
        {
            cv::arrowedLine(imTrack, curLeftPts[i], mapIt->second, cv::Scalar(0, 255, 0), 1, 8, 0, 0.2);
        }
    }

    //draw prediction
    /*
    for(size_t i = 0; i < predict_pts_debug.size(); i++)
    {
        cv::circle(imTrack, predict_pts_debug[i], 2, cv::Scalar(0, 170, 255), 2);
    }
    */
    //printf("predict pts size %d \n", (int)predict_pts_debug.size());

    //cv::Mat imCur2Compress;
    //cv::resize(imCur2, imCur2Compress, cv::Size(cols, rows / 2));
}

//SONG:根据3D的 predictPts 来更新2D predict_pts, 如果 predictPts 不含有指定的id项，
//则将对应的 prev_pts 放入 predict_pts 。
void FeatureTracker::setPrediction(map<int, Eigen::Vector3d> &predictPts)
{
    hasPrediction = true;
    predict_pts.clear();
    predict_pts_debug.clear();
    map<int, Eigen::Vector3d>::iterator itPredict;
    for (size_t i = 0; i < ids.size(); i++)
    {
        //printf("prevLeftId size %d prevLeftPts size %d\n",(int)prevLeftIds.size(), (int)prevLeftPts.size());
        int id = ids[i];
        itPredict = predictPts.find(id);
        if (itPredict != predictPts.end()) //找到了指定id的特征点。
        {
            Eigen::Vector2d tmp_uv;
            //SONG:EquidistantCamera::spaceToPlane
            //将3d坐标投影到像素坐标平面。
            m_camera[0]->spaceToPlane(itPredict->second, tmp_uv);
            predict_pts.push_back(cv::Point2f(tmp_uv.x(), tmp_uv.y()));
            predict_pts_debug.push_back(cv::Point2f(tmp_uv.x(), tmp_uv.y()));
        }
        else
            predict_pts.push_back(prev_pts[i]);
    }
}

//剔除干扰点。
//removePtsIds 存放的是需要剔除的特征点的id。
//status:0  剔除  status:1 保留
void FeatureTracker::removeOutliers(set<int> &removePtsIds)
{
    std::set<int>::iterator itSet;
    vector<uchar> status;
    for (size_t i = 0; i < ids.size(); i++)
    {
        itSet = removePtsIds.find(ids[i]);
        if(itSet != removePtsIds.end())
            status.push_back(0);
        else
            status.push_back(1);
    }

    reduceVector(prev_pts, status);
    reduceVector(ids, status);
    reduceVector(track_cnt, status);
}


cv::Mat FeatureTracker::getTrackImage()
{
    return imTrack;
}