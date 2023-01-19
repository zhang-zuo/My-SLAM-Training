//
// Created by zhangzuo on 22-8-19.
//
#include "LocalMap.h"
#include "KeyFrame.h"
#include "Optimizer.h"
#include "ShowRVIZ.h"


clock_t start, finish;
static long unsigned int mnId = 0;
ros::Publisher pub_mappoint;


KeyFrame* LocalMapping::GetKeyFrame(local_map::Track_LocalMap::Request &keyframe_msg)
{
    KeyFrame* msgCurrKeyFrame = new KeyFrame(mVocabulary);

    msgCurrKeyFrame->mTimeStamp = keyframe_msg.header.stamp.toSec();
//    ROS_INFO_STREAM("time: " << msgCurrKeyFrame->mTimeStamp);
    msgCurrKeyFrame->mid = mnId++;
//    ROS_INFO_STREAM("mid: " << msgCurrKeyFrame->mid);
    ROS_WARN_STREAM("local_map => get keyframe from feature_tracking! ");
    msgCurrKeyFrame->frameId = keyframe_msg.frameId;
    ROS_INFO_STREAM("local_map => frameId: " << msgCurrKeyFrame->frameId << "\tmid: " << msgCurrKeyFrame->mid);

    cv::Mat kf_pose = cv::Mat(keyframe_msg.KF_pose).reshape(0,4);
    msgCurrKeyFrame->SetPose(kf_pose);
//    ROS_INFO_STREAM("Pose: " << msgCurrKeyFrame->GetPose());

    cv::Mat kf_keypoints = cv::Mat(keyframe_msg.KF_keypoints).reshape(0,4);
//    ROS_INFO_STREAM("size: " << kf_keypoints.size << "msg: " << msg.KF_keypoints.size());
    for(int i = 0; i < keyframe_msg.KF_keypoints.size()/4; i++)
    {
        cv::Mat keypoint = kf_keypoints.col(i);
        cv::Point2f p2f = cv::Point2f(keypoint.at<float>(0),keypoint.at<float>(1));
//        ROS_INFO_STREAM("p2f: " << p2f);
        cv::KeyPoint p3k = cv::KeyPoint(p2f,
                                        0,
                                        keypoint.at<float>(2),
                                        0,
                                        keypoint.at<float>(3));
        msgCurrKeyFrame->mKeys.push_back(p3k);
    }

    //下面这里，如果直接给msgCurrKeyFrame->mDescriptors赋值，那么每次都会覆盖之前的描述子矩阵
    //必须使用copyto，这里也是找了好久
    int mDes_col = keyframe_msg.KF_descriptor.size()/32;
    cv::Mat mdes = cv::Mat(keyframe_msg.KF_descriptor).reshape(0,mDes_col);
    mdes.convertTo(mdes,CV_8U);
    mdes.copyTo(msgCurrKeyFrame->mDescriptors);
//    ROS_WARN_STREAM("mDescriptors: " << msgCurrKeyFrame->mDescriptors.size);
//    ROS_WARN_STREAM("mDes-*: " << &msgCurrKeyFrame->mDescriptors);

    msgCurrKeyFrame->KeyPointisMapPoint.resize(msgCurrKeyFrame->mKeys.size(), false);

    msgCurrKeyFrame->EarseAllMapPoint();
    if(keyframe_msg.mstate==0 || keyframe_msg.mstate==1)
    {
        cv::Mat kf_mappoints = cv::Mat(keyframe_msg.KF_mappoints).reshape(0,5);
        for(int j = 0; j < keyframe_msg.KF_mappoints.size()/5; j++)
        {
            cv::Mat mappoint_xyz = kf_mappoints.col(j).rowRange(0,3);
            int idx = kf_mappoints.at<float>(3,j);
            unsigned long mpid = kf_mappoints.at<float>(4,j);

            MapPoint* pMP;
            if(keyframe_msg.mstate == 0)
                pMP = new MapPoint(mappoint_xyz,mpid);
            else
            {
                auto findMapPoint = mDataBaseMapPoint.find(mpid);
//            ROS_WARN_STREAM("findMapPoint: " << findMapPoint->first);

                if(findMapPoint == mDataBaseMapPoint.end())
                {
                    ROS_ERROR_STREAM("local_map => without this mpid: " << mpid);
                    continue;
                }

                pMP = findMapPoint->second;

                //这里出现了意想不到的情况：指针没变，但是指针指向的内存出现了乱码（然而之前几次都是指针指向的内存都是没问题的），不知道是怎么回事。。。
                if(pMP->mid != mpid)
                {
                    ROS_ERROR_STREAM("BUG!!!");
                    //不能delete，因为后面说不定这个指针指向的内存有好使了。。。。
                    continue;
                }
            }

            pMP->AddObservation(msgCurrKeyFrame,idx);
            msgCurrKeyFrame->AddMapPoint(mpid,pMP,idx);
            msgCurrKeyFrame->KeyPointisMapPoint[idx] = true;

            if(keyframe_msg.mstate==0)
                mDataBaseMapPoint.insert(make_pair(mpid, pMP));

//            ROS_WARN_STREAM("pid: " << pMP->mid << "\tp: " << pMP->GetWorldPos() << "pMP: " << &pMP);
            if(j == (keyframe_msg.KF_mappoints.size()/5 -1))
                mMaxMapPointId = mpid;
        }

        cv::Mat cam = cv::Mat(keyframe_msg.Camera_config);
        fx = cam.at<float>(0);
        fy = cam.at<float>(1);
        cx = cam.at<float>(2);
        cy = cam.at<float>(3);

        cv::Mat K = cv::Mat::eye(3,3,CV_32F);
        K.at<float>(0,0) = fx;
        K.at<float>(1,1) = fy;
        K.at<float>(0,2) = cx;
        K.at<float>(1,2) = cy;
        K.copyTo(mk);
//        ROS_INFO_STREAM("mk: " << mk);
//        ROS_WARN_STREAM("pose: " << msgCurrKeyFrame->GetPose().reshape(0,1));
//        ROS_WARN_STREAM("kp: " << msgCurrKeyFrame->mKeys[120].pt.x <<
//                               " "    << msgCurrKeyFrame->mKeys[120].pt.y <<
//                               " angle: " << msgCurrKeyFrame->mKeys[120].angle <<
//                               " octave: " << msgCurrKeyFrame->mKeys[120].octave);
//        ROS_WARN_STREAM("kp: " << msgCurrKeyFrame->mKeys[360].pt.x <<
//                               " "    << msgCurrKeyFrame->mKeys[360].pt.y <<
//                               " angle: " << msgCurrKeyFrame->mKeys[360].angle <<
//                               " octave: " << msgCurrKeyFrame->mKeys[360].octave);
//        ROS_WARN_STREAM("des: " << msgCurrKeyFrame->mDescriptors.row(120));
    }
    else if(keyframe_msg.mstate == 2)
    {
        //idx mnid
//        KeyFrame* FindLastKeyFrame = *(mDataBaseKeyFrame.end()-1);
//        ROS_WARN_STREAM("FindLastKeyFrame-id: " << FindLastKeyFrame->mid);
//        auto mLastMapPoint = FindLastKeyFrame->GetmvpMapPoint();
        cv::Mat kf_mappoints = cv::Mat(keyframe_msg.KF_mappoints).reshape(0,2);
        ROS_WARN_STREAM("kf_mappoints.size: " << kf_mappoints.size);
        int num = 0;
        for(int j = 0; j < kf_mappoints.cols; j++)
        {
            int idx = (int)kf_mappoints.at<float>(0,j);
            unsigned long mpid = (unsigned long)kf_mappoints.at<float>(1,j);
//            ROS_WARN_STREAM("mpid: " << mpid);


            auto findMapPoint = mDataBaseMapPoint.find(mpid);
//            ROS_WARN_STREAM("findMapPoint: " << findMapPoint->first);

            if(findMapPoint == mDataBaseMapPoint.end())
            {
                ROS_ERROR_STREAM("local_map => without this mpid: " << mpid);
                continue;
            }

            MapPoint* pMP = findMapPoint->second;

            //这里出现了意想不到的情况：指针没变，但是指针指向的内存出现了乱码（然而之前几次都是指针指向的内存都是没问题的），不知道是怎么回事。。。
            if(pMP->mid != mpid)
            {
                ROS_ERROR_STREAM("BUG!!!");
                //不能delete，因为后面说不定这个指针指向的内存有好使了。。。。
                continue;
            }

//            ROS_DEBUG_STREAM("pMP: " << pMP);
//            ROS_DEBUG_STREAM("pMP-mid: " << pMP->mid << "\tmpid: " << mpid << "\tpMP: " << pMP << "\tidx: " << idx);
            pMP->AddObservation(msgCurrKeyFrame,idx);
//            auto a = pMP->observation;
//            for(auto it = a.begin(); it != a.end(); it++)
//            {
//                ROS_WARN_STREAM("observation-frameid: " << it->first->frameId << "\tmCurrFrameid: " << msgCurrKeyFrame->frameId);
//            }
            num++;
//            ROS_INFO_STREAM("local_map => receive MapPoints size: " << msgCurrKeyFrame->mvpMapPoint.size());
            msgCurrKeyFrame->AddMapPoint(mpid, pMP, idx);
            msgCurrKeyFrame->KeyPointisMapPoint[idx] = true;
//            ROS_DEBUG_STREAM("pMP-mid: " << pMP->mid << "\tmpid: " << mpid << "\tpos: " << pMP->GetWorldPos().reshape(0,1) << "\tpMP: " << pMP << "\tidx: " << idx);
        }
        ROS_WARN_STREAM("num: " << num);
    }

    if(keyframe_msg.mstate != 0)
    {
        cv::Mat img = cv::Mat(keyframe_msg.image).reshape(keyframe_msg.channel,keyframe_msg.height);
        img.copyTo(msgCurrKeyFrame->mKeyFrameImage);
    }
    ROS_INFO_STREAM("local_map => receive MapPoints size: " << msgCurrKeyFrame->mvpMapPoint.size());
//    ROS_WARN_STREAM("des: \n" << msgCurrKeyFrame->mDescriptors.rowRange(0,4));

    //初始化过程发来的keyframe
    if(keyframe_msg.mstate == 0)
        mInitKeyFrame = msgCurrKeyFrame;

    return msgCurrKeyFrame;
}


//req是接收到的数据　　　resp是需要发送出去的数据
bool LocalMapping::keyframe_callback(local_map::Track_LocalMap::Request &req,
                                     local_map::Track_LocalMap::Response &resp) {

    KeyFrame* mCurrKeyFrame = GetKeyFrame(req);

    start = ros::Time::now().toNSec();

    mCurrKeyFrame->ComputeBoW();
    mCurrKeyFrame->UpdateCoviewing();

    //跟踪过程发来的keyframe
    if(mCurrKeyFrame->mid > 1)
    {
        vector<KeyFrame*> mCurrCoviewingKeyFrame;
        mCurrCoviewingKeyFrame = FindLocalCoviewing(mCurrKeyFrame, 15);

        vector<MapPoint*> CreateMapPoints;
        for(KeyFrame* mCoviewingKeyFrame : mCurrCoviewingKeyFrame)
        {
            ROS_WARN_STREAM("mCoviewId: " << mCoviewingKeyFrame->mid << "\tmCurrId: " << mCurrKeyFrame->frameId);
            //对共视帧和当前关键帧进行视差检查，如果视差太小，那么不适合进行三角化
            //通过跟踪的地图点进行视差检查
//                    if(!CheckKeyFrameParallax(mCurrKeyFrame,mCoviewingKeyFrame))
//                        continue;

            //这里测试使用极线约束后，非常耗时，所以暂时不使用
            cv::Mat F21;
            ComputeF21(mCoviewingKeyFrame, mCurrKeyFrame, F21);

            //first共视关键帧索引 second当前关键帧索引
            //这里的匹配，会出现一对多和多对一的情况，需要修改
            vector<pair<int, int>> matchesIdx;
            SearchForLocalMap(mCurrKeyFrame, mCoviewingKeyFrame, F21, matchesIdx);

            if(matchesIdx.size()==0)
                continue;

            vector<cv::Point3f> mCurr_Coview_MP;
            vector<pair<int, int>> matchMP2KP;
            Trangulation(mCurrKeyFrame,mCoviewingKeyFrame,matchesIdx,mCurr_Coview_MP, matchMP2KP);

            for(int i = 0; i < mCurr_Coview_MP.size(); i++)
            {
                cv::Point3f MPw = mCurr_Coview_MP[i];
                cv::Mat p3d_w = (cv::Mat_<float>(3,1) << MPw.x, MPw.y, MPw.z);
                MapPoint* pMP = new MapPoint(p3d_w);
                int mCoviewIdx = matchMP2KP[i].first;
                int mCurrIdx = matchMP2KP[i].second;

                pMP->AddObservation(mCoviewingKeyFrame,mCoviewIdx);
                pMP->AddObservation(mCurrKeyFrame,mCurrIdx);
                mCoviewingKeyFrame->AddMapPoint(pMP->mid,pMP,mCoviewIdx);
                mCurrKeyFrame->AddMapPoint(pMP->mid,pMP,mCurrIdx);
                mCoviewingKeyFrame->KeyPointisMapPoint[mCoviewIdx] = true;
                mCurrKeyFrame->KeyPointisMapPoint[mCurrIdx] = true;

                CreateMapPoints.push_back(pMP);

                if(mMaxMapPointId < pMP->mid)
                    mMaxMapPointId = pMP->mid;
//                ROS_DEBUG_STREAM("pMP-mid: " << pMP->mid << "\tp: " << pMP->GetWorldPos());
            }
        }

        mCurrKeyFrame->UpdateCoviewing();

        vector<KeyFrame*> LocalMapKeyFrame;
        LocalMapKeyFrame = FindLocalKeyFrame(mCurrKeyFrame, 10);

        int LocalMapNum = 6;
        if(LocalMapKeyFrame.size() < LocalMapNum)
            LocalMapNum = LocalMapKeyFrame.size();

        for(int j = 0; j < LocalMapNum; j++)
            mCurrKeyFrame->mCoviewFrameId.push_back(LocalMapKeyFrame[j]->frameId);

        //g2o优化
        LocalMapBA(LocalMapKeyFrame, mCurrKeyFrame, mCurrCoviewingKeyFrame, CreateMapPoints, mk);

        //对于外点，应该再当前帧去掉
        //那么是否应该去掉观测呢。好像也不用
        auto mCurrMapPoints = mCurrKeyFrame->GetmvpMapPoint();
        for(auto it = mCurrMapPoints.begin(); it != mCurrMapPoints.end(); it++)
        {
            unsigned long id = it->first;
            if(it->second.first->is_outlier)
            {
                mCurrKeyFrame->KeyPointisMapPoint[it->second.second] = false;
//                if(!it->second.first)
//                    continue;
//
//                delete it->second.first;
                mCurrKeyFrame->EarseOneMapPoint(id);
                it->second.first->EarseOneObservation(mCurrKeyFrame);
            }
            else
                mDataBaseMapPoint.insert(make_pair(id,it->second.first));
            //如果id重复，就会自动放弃插入
            //因为mDataBaseMapPoint中存放的是MapPoint的指针，因此修改了指针内的东西不需要去覆盖这个mDataBaseMapPoint

        }

        ROS_INFO_STREAM("local_map => mCurrMapPoints size: " << mCurrKeyFrame->GetmvpMapPoint().size());

        //优化之后就把信息发送出去
        SendToTracking(mCurrKeyFrame, resp);

    }

    mDataBaseKeyFrame.push_back(mCurrKeyFrame);
//            delete mCurrKeyFrame;

    finish = ros::Time::now().toNSec();
    ROS_DEBUG_STREAM("localmap-runtime: " << (double)(finish- start)/10e6 << " ms");

    cv::Mat mCurrR = mCurrKeyFrame->GetPose().rowRange(0,3).colRange(0,3);
    cv::Mat mCurrt = mCurrKeyFrame->GetPose().rowRange(0,3).col(3);
    cv::Mat mCurrO = -mCurrR.t() * mCurrt;
    ROS_WARN_STREAM("local_map => mCurrKeyFrame: " << mCurrKeyFrame->frameId << "\tmOw: " << mCurrO.reshape(0,1));

    ShowRVIZ(mDataBaseKeyFrame,mDataBaseMapPoint,mCurrKeyFrame);

    return true;
}


void LocalMapping::SendToTracking(KeyFrame* mCurrKF, local_map::Track_LocalMap::Response &resp)
{
    local_map::Track_LocalMap msg;
    resp.mstate = 2;

    resp.header.stamp = (ros::Time)mCurrKF->mTimeStamp;
    resp.header.frame_id = "world";

    resp.frameId = mCurrKF->frameId;

//    ROS_WARN_STREAM("mid: " << mCurrKF->mid <<
//                    "\tframeid: " << mCurrKF->frameId <<
//                    "\tseq: " << msg.header.seq);

    resp.KF_pose = mCurrKF->GetPose().reshape(0,1);

    auto mmvpMapPoints = mCurrKF->GetmvpMapPoint();
    cv::Mat KF_mappoints(5,mmvpMapPoints.size(),CV_32F);
    int num = 0;
    for(auto it = mmvpMapPoints.begin(); it != mmvpMapPoints.end(); it++)
    {
        MapPoint* pMP = it->second.first;

        KF_mappoints.at<float>(0,num) = pMP->GetWorldPos().at<float>(0);
        KF_mappoints.at<float>(1,num) = pMP->GetWorldPos().at<float>(1);
        KF_mappoints.at<float>(2,num) = pMP->GetWorldPos().at<float>(2);
        KF_mappoints.at<float>(3,num) = (float)it->second.second;
        KF_mappoints.at<float>(4,num) = (float)it->first;
        num++;
    }
    resp.KF_mappoints = KF_mappoints.reshape(0,1);

    cv::Mat mCoview(1,mCurrKF->mCoviewFrameId.size()+1,CV_32F);
    mCoview.at<float>(0) = mCurrKF->mCoviewFrameId.size();
    ROS_WARN_STREAM("mCoviewFrameId.size: " << mCoview.at<float>(0));
    for(int i = 0; i < mCurrKF->mCoviewFrameId.size(); i++)
    {
        mCoview.at<float>(i+1) = mCurrKF->mCoviewFrameId[i];
//        ROS_WARN_STREAM("CoviewId: " << mCurrKF->mCoviewFrameId[i]);
    }
    resp.CoviewId = mCoview.reshape(0,1);


//    pub_mappoint.publish(msg);
//    ROS_WARN_STREAM("local_map => publish to feature_tracking! " <<  mCurrKF->frameId);
}

//这里只是再回调函数中处理，其实不应该再回电函数中处理这些数据
//应该单独起一个线程，用于处理数据，回调函数只是用于接受数据
int main(int argc, char** argv){

    ros::init(argc, argv, "local_map");
    ros::NodeHandle n;
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    RegisterPub(n);

//    pub_mappoint = n.advertise<local_map::Track_LocalMap>("local2feature", 1000);

    LocalMapping localmap(n);

//    ros::Subscriber sub_keyframe = n.subscribe("feature2local", 1000, &LocalMapping::keyframe_callback, &localmap);
    ros::ServiceServer server = n.advertiseService("Track_LocalMap",&LocalMapping::keyframe_callback,&localmap);

//    std::thread t(&LocalMapping::process, &localmap);
//    t.detach();

    ros::spin();

    return 0;
}