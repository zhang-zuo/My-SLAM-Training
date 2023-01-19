
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "Tracking.h"


clock_t start, finish;
std::string IMAGE_TOPIC;
image_transport::Publisher pub_img;
ros::ServiceClient client_keyframe;
ros::Publisher pub_path;
nav_msgs::Path TrackPath;

yamlfile yamlFile;
std::mutex m_map;
std::mutex m_frame;

Tracking::Tracking(ros::NodeHandle &n)
{

    //读取配置文件
    std::string config_file;
    if(n.getParam("config_file", config_file))
    {
        ROS_DEBUG_STREAM("Loaded config_file: " << config_file);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load config_file " << config_file);
        n.shutdown();
    }

    cv::FileStorage fileSetting(config_file, cv::FileStorage::READ);
    if(!fileSetting.isOpened())
    {
        ROS_ERROR_STREAM("Failed to open config_file ");
        n.shutdown();
    }

    fileSetting["Image.type"] >> IMAGE_TOPIC;
    yamlFile.fx = fileSetting["Camera.fx"];
    yamlFile.fy = fileSetting["Camera.fy"];
    yamlFile.cx = fileSetting["Camera.cx"];
    yamlFile.cy = fileSetting["Camera.cy"];

    yamlFile.k1 = fileSetting["Camera.k1"];
    yamlFile.k2 = fileSetting["Camera.k2"];
    yamlFile.p1 = fileSetting["Camera.p1"];
    yamlFile.p2 = fileSetting["Camera.p2"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = yamlFile.fx;
    K.at<float>(1,1) = yamlFile.fy;
    K.at<float>(0,2) = yamlFile.cx;
    K.at<float>(1,2) = yamlFile.cy;
    K.copyTo(yamlFile.mk);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = yamlFile.k1;
    DistCoef.at<float>(1) = yamlFile.k2;
    DistCoef.at<float>(2) = yamlFile.p1;
    DistCoef.at<float>(3) = yamlFile.p2;
    const float k3 = fileSetting["Camera.k3"];
    //有些相机的畸变系数中会没有k3项
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(yamlFile.mDistCoef);

    yamlFile.nFeature = fileSetting["ORBextractor.nFeatures"];
    yamlFile.scaleFactor = fileSetting["ORBextractor.scaleFactor"];
    yamlFile.nLevels = fileSetting["ORBextractor.nLevels"];
    yamlFile.iniThFAST = fileSetting["ORBextractor.iniThFAST"];   //20
    yamlFile.minThFAST = fileSetting["ORBextractor.minThFAST"];   //7

    ROS_DEBUG_STREAM("mk: "          << yamlFile.mk.reshape(0,1));
    ROS_DEBUG_STREAM("dis: "         << yamlFile.mDistCoef.reshape(0,1));
    ROS_DEBUG_STREAM("nFeature: "    << yamlFile.nFeature);
    ROS_DEBUG_STREAM("scaleFactor: " << yamlFile.scaleFactor);
    ROS_DEBUG_STREAM("nLevels: "     << yamlFile.nLevels);
    ROS_DEBUG_STREAM("iniThFAST: "   << yamlFile.iniThFAST);
    ROS_DEBUG_STREAM("minThFAST: "   << yamlFile.minThFAST);
    ROS_INFO_STREAM("---------------------------------------------");
}


void Tracking::SendKeyFrame()
{
    //将关键帧数据发送出去
    if(FindKeyFrame)
    {
//        ROS_DEBUG_STREAM("*****************************************************");
        ROS_INFO_STREAM("feature_tracking => SendMsg to localmap");
        //初始化结束后，发送给local_map共两帧数据，所以一共发送两次
        if(Init_send)
        {
            vector<Frame *> InitCurrFrame;
            InitCurrFrame.push_back(&mInitFrame);
            InitCurrFrame.push_back(&mCurrentFrame);

            for(int i = 0; i < InitCurrFrame.size(); i++)
            {
                //当前状态位，0,1指的是初始化的两帧
                msg_keyframe.request.mstate = i;

                //KeyFrame1.id
                //KeyFrame1.timestamp
                msg_keyframe.request.header.seq = InitCurrFrame[i]->mid;
                msg_keyframe.request.header.stamp = (ros::Time)InitCurrFrame[i]->mTimeStamp;
                msg_keyframe.request.header.frame_id = "world";

                msg_keyframe.request.frameId = InitCurrFrame[i]->mid;

                //KeyFrame1.pose
                msg_keyframe.request.KF_pose = InitCurrFrame[i]->GetPose().reshape(0,1);

                //KeyFrame1.mKeys -> pt angle octave
                cv::Mat KF_ketpoints(4,InitCurrFrame[i]->mKeys.size(),CV_32F);
                for(int j0 = 0; j0 < InitCurrFrame[i]->mKeys.size(); j0++)
                {
                    KF_ketpoints.at<float>(0,j0) = (float)InitCurrFrame[i]->mKeys[j0].pt.x;
                    KF_ketpoints.at<float>(1,j0) = (float)InitCurrFrame[i]->mKeys[j0].pt.y;
                    KF_ketpoints.at<float>(2,j0) = (float)InitCurrFrame[i]->mKeys[j0].angle;
                    KF_ketpoints.at<float>(3,j0) = (float)InitCurrFrame[i]->mKeys[j0].octave;
                }
                msg_keyframe.request.KF_keypoints = KF_ketpoints.reshape(0,1);
                ROS_INFO_STREAM("KF_keypoints.size: " << msg_keyframe.request.KF_keypoints.size());

                //KeyFrame1.mdescriptor -> 1200x32
                msg_keyframe.request.KF_descriptor = InitCurrFrame[i]->mDescriptors.reshape(0,1);
                ROS_INFO_STREAM("KF_descriptor.size: " << msg_keyframe.request.KF_descriptor.size());

                //KeyFrame1.mvpMapPoint -> MapPoint.pose idx mnid
                vector<pair<MapPoint*,int>> mvpMP = InitCurrFrame[i]->GetmvpMapPoint();
                cv::Mat KF_mappoints(5,mvpMP.size(),CV_32F);
                ROS_INFO_STREAM("feature_tracking => send MapPoints size: " << mvpMP.size());
                for(int j1 = 0; j1 < mvpMP.size(); j1++)
                {
                    MapPoint* pMP = mvpMP[j1].first;

                    KF_mappoints.at<float>(0,j1) = pMP->GetWorldPos().at<float>(0);
                    KF_mappoints.at<float>(1,j1) = pMP->GetWorldPos().at<float>(1);
                    KF_mappoints.at<float>(2,j1) = pMP->GetWorldPos().at<float>(2);
                    KF_mappoints.at<float>(3,j1) = (float)mvpMP[j1].second;
                    KF_mappoints.at<float>(4,j1) = (float)pMP->mid;
//                    ROS_WARN_STREAM("pMP.id: " << pMP->mid << "\tpos: " << pMP->GetWorldPos().reshape(0,1));
                }
                msg_keyframe.request.KF_mappoints = KF_mappoints.reshape(0,1);
//                ROS_INFO_STREAM("KF_mappoints.size: " << msg_keyframe.KF_mappoints.size());

                //相机参数
                cv::Mat cam_config = (cv::Mat_<float>(1,4) << yamlFile.fx, yamlFile.fy, yamlFile.cx, yamlFile.cy);
                msg_keyframe.request.Camera_config = cam_config;

                if(i == 1)
                {
                    int h = InitCurrFrame[i]->mFrameImg.rows;
                    int w = InitCurrFrame[i]->mFrameImg.cols;
                    int c = InitCurrFrame[i]->mFrameImg.channels();
                    msg_keyframe.request.height = h;
                    msg_keyframe.request.width = w;
                    msg_keyframe.request.channel = c;
                    msg_keyframe.request.image = vector<uint8_t>(InitCurrFrame[i]->mFrameImg.reshape(1,h*w*c));
                }

//                ROS_WARN_STREAM("pose: " << InitCurrFrame[i]->GetPose().reshape(0,1));
//                ROS_WARN_STREAM("kp: " << InitCurrFrame[i]->mKeys[120].pt.x <<
//                                " "    << InitCurrFrame[i]->mKeys[120].pt.y <<
//                                " angle: " << InitCurrFrame[i]->mKeys[120].angle <<
//                                " octave: " << InitCurrFrame[i]->mKeys[120].octave);
//                ROS_WARN_STREAM("kp: " << InitCurrFrame[i]->mKeys[360].pt.x <<
//                                       " "    << InitCurrFrame[i]->mKeys[360].pt.y <<
//                                       " angle: " << InitCurrFrame[i]->mKeys[360].angle <<
//                                       " octave: " << InitCurrFrame[i]->mKeys[360].octave);
//                ROS_WARN_STREAM("des: " << InitCurrFrame[i]->mDescriptors.row(120));
//                auto xxp = mvpMP[12].first;
//                ROS_WARN_STREAM("pMP: " << "id: " << xxp->mid
//                                        << "\tpos: " << xxp->GetWorldPos().reshape(0,1)
//                                        << "\tidx: " << mvpMP[12].second);
                if(i == 1)
                {
                    //这块的赋值，是深拷贝还是浅拷贝，有点懵  TODO
                    //以防万一，我直接new一个新的，然后传递指针
                    mapKeyFrame.insert(make_pair(mCurrentFrame.mid, InitCurrFrame[i]));
                }

                client_keyframe.call(msg_keyframe);

                ROS_INFO_STREAM("feature_tracking => publish to local_map!" << i);
            }
            Init_send = false;
        }
        else
        {
            //mstate=2指的是track过程，一切正常，暂时还没有别的用处
            msg_keyframe.request.mstate = 2;

            //KeyFrame1.id
            //KeyFrame1.timestamp
            msg_keyframe.request.header.seq = mCurrentFrame.mid;
            msg_keyframe.request.header.stamp = (ros::Time)mCurrentFrame.mTimeStamp;
            msg_keyframe.request.header.frame_id = "world";

            msg_keyframe.request.frameId = mCurrentFrame.mid;

            //KeyFrame1.pose
            msg_keyframe.request.KF_pose = mCurrentFrame.GetPose().reshape(0,1);

            //KeyFrame1.mKeys -> pt angle octave
            cv::Mat KF_ketpoints(4,mCurrentFrame.mKeys.size(),CV_32F);
            for(int j0 = 0; j0 < mCurrentFrame.mKeys.size(); j0++)
            {
                KF_ketpoints.at<float>(0,j0) = (float)mCurrentFrame.mKeys[j0].pt.x;
                KF_ketpoints.at<float>(1,j0) = (float)mCurrentFrame.mKeys[j0].pt.y;
                KF_ketpoints.at<float>(2,j0) = (float)mCurrentFrame.mKeys[j0].angle;
                KF_ketpoints.at<float>(3,j0) = (float)mCurrentFrame.mKeys[j0].octave;
            }
            msg_keyframe.request.KF_keypoints = KF_ketpoints.reshape(0,1);
//            ROS_INFO_STREAM("KF_keypoints.size: " << msg_keyframe.KF_keypoints.size());

            //KeyFrame1.mdescriptor -> 32x1200
            msg_keyframe.request.KF_descriptor = mCurrentFrame.mDescriptors.reshape(0,1);
//            ROS_INFO_STREAM("KF_descriptor.size: " << msg_keyframe.KF_descriptor.size());

            //KeyFrame1.mvpMapPoint -> idx mnid
            map<unsigned long, MapPoint*> mDataCurrMapPoints;
            vector<pair<MapPoint*,int>> mvpMP = mCurrentFrame.GetmvpMapPoint();
            cv::Mat KF_mappoints(2,mvpMP.size(),CV_32F);
            ROS_INFO_STREAM("feature_tracking => send MapPoints size: " << mvpMP.size());
            for(int j1 = 0; j1 < mvpMP.size(); j1++)
            {
                MapPoint* pMP = mvpMP[j1].first;

                KF_mappoints.at<float>(0,j1) = (float)mvpMP[j1].second;
                KF_mappoints.at<float>(1,j1) = (float)pMP->mid;

                mDataCurrMapPoints.insert(make_pair(pMP->mid, pMP));
//                ROS_WARN_STREAM("pMP.id: " << pMP->mid << "\tpos: " << pMP->GetWorldPos().reshape(0,1));
            }
            msg_keyframe.request.KF_mappoints = KF_mappoints.reshape(0,1);
//            ROS_INFO_STREAM("KF_mappoints.size: " << msg_keyframe.KF_mappoints.size());

            int h = mCurrentFrame.mFrameImg.rows;
            int w = mCurrentFrame.mFrameImg.cols;
            int c = mCurrentFrame.mFrameImg.channels();
            msg_keyframe.request.height = h;
            msg_keyframe.request.width = w;
            msg_keyframe.request.channel = c;
            msg_keyframe.request.image = vector<uint8_t>(mCurrentFrame.mFrameImg.reshape(1,h*w*c));

            ROS_INFO_STREAM("feature_tracking => publish to local_map!" << mCurrentFrame.mid);

            //阻塞接收数据
            if(client_keyframe.call(msg_keyframe))
            {
                //接收到localmap的数据，主要是地图点，用于tracking
                ROS_WARN_STREAM("feature_tracking => get localmap");
                if(msg_keyframe.response.mstate == 2)
                {
                    //下面这块需要测试
//        if(!NewKeyFrame.first && !mKeyFrame)
//        {
//            delete mapKeyFrame.find(NewKeyFrame.second)->second;
//            mapKeyFrame.erase(NewKeyFrame.second);
//        }
                    unsigned long mid = msg_keyframe.response.frameId;
                    ROS_WARN_STREAM("mid: " << mid);

                    if(mCurrentFrame.mid != mid)
                    {
                        ROS_ERROR_STREAM("get KeyFrame msg-mid error!!!");
                        return;
                    }

                    //关键帧优化后的位姿，其实没用
                    cv::Mat kf_pose = cv::Mat(msg_keyframe.response.KF_pose).reshape(0,4);
//        ROS_WARN_STREAM("src-pose: " << mNewKeyFrame->GetPose());
//        ROS_WARN_STREAM("des-pose: " << kf_pose);
                    mCurrentFrame.SetPose(kf_pose);
                    //重新更新关键帧再localmap三角化的地图点，这是重要的

                    //这里只要delete，就容易出现非法操作，不知道什么问题
//                    mCurrentFrame.DeleteAllMapPoint();
                    mCurrentFrame.EarseAllMapPoint();
                    cv::Mat kf_mappoints = cv::Mat(msg_keyframe.response.KF_mappoints).reshape(0,5);
                    for(int j = 0; j < msg_keyframe.response.KF_mappoints.size()/5; j++)
                    {
                        cv::Mat mappoint_xyz = kf_mappoints.col(j).rowRange(0,3);
                        int idx = kf_mappoints.at<float>(3,j);
                        int mpid = kf_mappoints.at<float>(4,j);

                        //地图点分为两种，一种track过程中原有的，一种localmap新建的
                        MapPoint* pMP;
                        auto mResponseMappoint = mDataCurrMapPoints.find(mpid);
                        if(mResponseMappoint != mDataCurrMapPoints.end())
                            pMP = mResponseMappoint->second;
                        else
                            pMP = new MapPoint(mappoint_xyz,mpid);

                        mCurrentFrame.AddMapPoint(pMP,idx);
//                        ROS_WARN_STREAM("mpid: " << mpid << "\tpos: " << pMP->GetWorldPos().reshape(0,1));
                    }
//        ROS_WARN_STREAM("size: " << mNewKeyFrame->mvpMapPoint.size());
                    //标志位
//                    NewKeyFrame.first = true;
//                    NewKeyFrame.second = mid;

                    cv::Mat mCoview = cv::Mat(msg_keyframe.response.CoviewId);
                    int nCoview = (int)mCoview.at<float>(0);
                    ROS_WARN_STREAM("nCoview: " << nCoview);
                    for(int i = 0; i < nCoview; i++)
                    {
                        mCurrentFrame.mCoviewFrameId.push_back((int)mCoview.at<float>(i+1));
//                        ROS_WARN_STREAM("mCoviewFrameId: " << mCurrentFrame.mCoviewFrameId[i]);
                    }
                }
            }
            else
                ROS_ERROR_STREAM("not Get LocalMap Msg!!!");

            Frame *Newkey = new Frame(mCurrentFrame);
            //这块的赋值，是深拷贝还是浅拷贝，有点懵  TODO
            //以防万一，我直接new一个新的，然后传递指针
            mapKeyFrame.insert(make_pair(mCurrentFrame.mid, Newkey));

        }
        FindKeyFrame = false;
//        ROS_DEBUG_STREAM("*****************************************************");
//        ros::Duration(0.5).sleep();

        cv::Mat mCurrR = mCurrentFrame.GetPose().rowRange(0,3).colRange(0,3);
        cv::Mat mCurrt = mCurrentFrame.GetPose().rowRange(0,3).col(3);
        cv::Mat mCurrO = -mCurrR.t() * mCurrt;
        ROS_WARN_STREAM("feature_tracking => mCurrFrame: " << mCurrentFrame.mid <<  "\tmOw: " << mCurrO.reshape(0,1));
    }
}



void Tracking::img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{

    //通过cv_bridge将sensor_msgs的图像数据转换为Mat类型
    //其中MONO8指的就是将图像转换位CV_8UC1（灰度图），BGR8是CV_8UC3（彩色图）
    ROS_DEBUG_ONCE("receive dataset!");
    cv_bridge::CvImageConstPtr ptr;
    ptr = cv_bridge::toCvCopy(img_msg,sensor_msgs::image_encodings::BGR8);
    cv::Mat img_color = ptr->image;
    //获得每张图像的时间戳
    double time_img = img_msg->header.stamp.toSec();

    m_frame.lock();
    mListNewFrame.push_back(make_pair(img_color,time_img));
    m_frame.unlock();
}


void Tracking::process()
{

    while(1)
    {
        cv::Mat ImgColor;
        double ImgTime;
        bool IsGetNewImage = GetNewFrame(ImgColor, ImgTime);

        if(IsGetNewImage)
        {
            start = ros::Time::now().toNSec();

            mCurrentFrame = Frame(ImgColor, ImgTime);
            ROS_WARN_STREAM("mCurr.mid: " << mCurrentFrame.mid);

            //单目初始化
            if(!InitFinish)
                MonoInitialization();
            else
            {
                if(mLastFrame.is_keyframe)
                    mCurrentFrame.mReferKeyFrameId = mLastFrame.mid;
                else
                    mCurrentFrame.mReferKeyFrameId = mLastFrame.mReferKeyFrameId;

                Track();

                if(mCurrentFrame.mid-mInitFrame.mid < 15)
                {
                    cv::Mat R = mCurrentFrame.GetPose().rowRange(0,3).colRange(0,3);
                    cv::Mat t = mCurrentFrame.GetPose().rowRange(0,3).col(3);
                    cv::Mat O = -R*t;
                    ROS_WARN_STREAM("Track-pos: " << O.reshape(0,1));
                }

                if(mCurrTrackMapPointSize < 15)
                    mCurrentFrame.SetPose(mLastFrame.GetPose());

                LocalMapTrack();

                if(mCurrentFrame.mid-mInitFrame.mid < 15)
                {
                    cv::Mat R = mCurrentFrame.GetPose().rowRange(0,3).colRange(0,3);
                    cv::Mat t = mCurrentFrame.GetPose().rowRange(0,3).col(3);
                    cv::Mat O = -R*t;
                    ROS_WARN_STREAM("LocalMapTrack-pos: " << O.reshape(0,1));
                }
//                if(mCurrentFrame.GetmvpMapPoint().size() < 10)
//                    mCurrentFrame.SetPose(mLastFrame.GetPose());
            }


            //需要添加一个判断是否关键帧的函数
            //1.设置如果跟踪的地图点数量太少了，那么就赶快创建新的关键帧，赶紧三角化新的地图点
            //2.如果上一帧是关键帧，那么当前帧不应该是关键帧
            //使用map保存每一个关键帧，但是用完了关键帧的地图点之后，就会将其删除，因为tracking过程不需要
            //所以这个map中也就存储了一两个，就销毁了
//    if(InitFinish && mCurrentFrame.mvpMapPoint.size() <= 70 && !mLastFrame.is_keyframe)

            ROS_WARN_STREAM("mCurrTrackMapPointSize: " << mCurrTrackMapPointSize << "\tmLastFrame: " << mLastFrame.GetmvpMapPoint().size());
            if( InitFinish && ((mCurrentFrame.mvpMapPoint.size() <= 70)
                            || ((float)mCurrTrackMapPointSize/(float)mLastFrame.GetmvpMapPoint().size() < 0.35)
                            || ((mCurrentFrame.mid-mInitFrame.mid<20) && ((float)mCurrTrackMapPointSize/(float)mLastFrame.GetmvpMapPoint().size() < 0.4))
                            ))
            {
                ROS_WARN_STREAM("feature_tracking => FindKeyFrame");
                ROS_WARN_STREAM("size: " << mCurrentFrame.mvpMapPoint.size());
                ROS_WARN_STREAM("/size: " << (((float)mCurrTrackMapPointSize)/((float)mLastFrame.GetmvpMapPoint().size())));
                FindKeyFrame = true;
                mCurrentFrame.is_keyframe = true;
            }

            //下面计算相对速度
            if(InitFinish)
            {
                cv::Mat mLastT = mLastFrame.GetPose();
                cv::Mat mLastR = mLastT.rowRange(0,3).colRange(0,3);
                cv::Mat mLastt = mLastT.rowRange(0,3).col(3);
                cv::Mat mLastTwc = cv::Mat::eye(4,4,CV_32F);
                mLastTwc.rowRange(0,3).colRange(0,3) = mLastR.t();
                mLastTwc.rowRange(0,3).col(3) = -mLastR.t() * mLastt;

//        ROS_INFO_STREAM("mLastR: " << mLastR);
//        ROS_INFO_STREAM("mLastt: " << mLastt);
//        ROS_INFO_STREAM("mLastTwc: " << mLastTwc);
                mVelocity = mCurrentFrame.GetPose() * mLastTwc;
            }

            SendKeyFrame();

//            if(FindKeyFrame)
//            {
////                TrackPath.poses.clear();
//                geometry_msgs::PoseStamped pose_stamped;
//                std_msgs::Header header;
//                header.frame_id = "world";
//                header.stamp = (ros::Time)mCurrentFrame.mTimeStamp;
//
//                cv::Mat R = mCurrentFrame.GetPose().rowRange(0,3).colRange(0,3);
//                cv::Mat t = mCurrentFrame.GetPose().rowRange(0,3).col(3);
//                cv::Mat O = -R*t;
//
//                pose_stamped.header = header;
//                pose_stamped.header.frame_id = "world";
//                pose_stamped.pose.position.x = O.at<float>(0);
//                pose_stamped.pose.position.y = O.at<float>(1);
//                pose_stamped.pose.position.z = O.at<float>(2);
//                TrackPath.poses.push_back(pose_stamped);
//                TrackPath.header = header;
//                TrackPath.header.frame_id = "world";
//                pub_path.publish(TrackPath);
//            }

            //当前帧的东西应该都处理完了，把当前帧数据给上一帧
            mLastFrame = Frame(mCurrentFrame);

            //不往local_map发送的,可视化处理
            //将画有跟踪点的图像发送往rviz
            if(InitFinish)
            {
                vector<pair<MapPoint*, int>> mvpMP = mCurrentFrame.GetmvpMapPoint();
                for(int i = 0; i < mvpMP.size(); i++)
                {
                    auto p2d = mCurrentFrame.mKeys[mvpMP[i].second];
                    circle(mCurrentFrame.mFrameImg, cv::Point(p2d.pt.x,p2d.pt.y), 2, cv::Scalar(0, 0, 255), -1);
                }

//    cv::namedWindow("show_kitii", CV_WINDOW_AUTOSIZE);
//    cv::imshow("show_kitii", img_color);
//    cv::waitKey(20);

                //发送画有跟踪点的图像，主要是再rviz中显示
                msg_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", mCurrentFrame.mFrameImg).toImageMsg();
                pub_img.publish(msg_image);
//    ROS_WARN_STREAM("++++++++++++++++++++++++++++++++++++++++++++++++"
//                    "++++++++++++++++++++++++++++++++++++++++++++++++");
            }

            finish = ros::Time::now().toNSec();
            ROS_DEBUG_STREAM("feature_tracking-runtime: " << (double)(finish- start)/10e6 << " ms");

        }
    }

}


int main(int argc, char** argv)
{
    //创建节点
    ros::init(argc, argv, "feature_tracking");
    ros::NodeHandle n;
    //设置log等级为DEBUG
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    //发送出去的画有跟踪点的图像发送出去
    image_transport::ImageTransport it(n);
    pub_img = it.advertise("feature_img",1000);
    //将关键帧数据发送出去
    client_keyframe = n.serviceClient<feature_tracking::Track_LocalMap>("Track_LocalMap");
    ros::service::waitForService("Track_LocalMap");
    //这里实际上就是读取了配置文件的内容，得到相机数据等
    Tracking track(n);

    //接收到rosbag的图像数据的callback
    ros::Subscriber sub_img = n.subscribe(IMAGE_TOPIC, 10000, &Tracking::img_callback, &track);
//    pub_path = n.advertise<nav_msgs::Path>("TrackPath", 1000);

//    ros::Subscriber sub_localmap = n.subscribe("local2feature", 1000, &Tracking::localmap_callback, &track);
    std::thread t(&Tracking::process, &track);
    t.detach();

    ros::spin();

//    ros::MultiThreadedSpinner spinner(2);
//    spinner.spin();

    return 0;
}


