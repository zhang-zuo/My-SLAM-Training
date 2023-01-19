//
// Created by zhangzuo on 22-8-19.
//

#ifndef LOCAL_MAP_LOCALMAP_H
#define LOCAL_MAP_LOCALMAP_H

#include "KeyFrame.h"
#include <iostream>
#include "time.h"
#include "ros/ros.h"
#include <opencv2/core/core.hpp>
#include "opencv2/opencv.hpp"
#include "local_map/Track_LocalMap.h"
#include "MapPoint.h"
#include <map>
#include <set>
#include "Eigen/Core"
#include "DBoW2/DBoW2/FORB.h"
#include "DBoW2/DBoW2/TemplatedVocabulary.h"
#include <thread>
#include "mutex"
#include <algorithm>

using namespace std;

typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB> ORBVocabulary;

class KeyFrame;
class MapPoint;

class LocalMapping{
public:
    LocalMapping();
    LocalMapping(ros::NodeHandle &n);

    void process();

    bool keyframe_callback(local_map::Track_LocalMap::Request &req,local_map::Track_LocalMap::Response &resp);

    //这个是找到当前关键帧的num数量的共视关键帧
    static vector<KeyFrame*> FindLocalCoviewing(KeyFrame *mCKeyFrame, int num);
    //这个是找到当前关键帧的权重为weight的共视关键帧，也就是局部地图的关键帧
    static vector<KeyFrame*> FindLocalKeyFrame(KeyFrame *mCKeyFrame, int weight);

    bool CheckKeyFrameParallax(KeyFrame* mCurrKF, KeyFrame* mCoviewKF);
    void SearchForLocalMap(KeyFrame* mCurrKF, KeyFrame* mCoviewKF, cv::Mat &F21, vector<pair<int, int>> &matchIndex);
    int DescriptorDistance(const cv::Mat &a, const cv::Mat &b);

    void ComputeF21(KeyFrame* mCoviewKF, KeyFrame* mCurrKF, cv::Mat &F21);
    bool CheckEpipolarLine(cv::KeyPoint &kp1, cv::KeyPoint &kp2, cv::Mat &F21);
    void ComputeThreeMaxima(vector<pair<int, pair<int, int>>>* histo, const int L, int &ind1, int &ind2, int &ind3);

    void Trangulation(KeyFrame* mCurrKF, KeyFrame* mCoviewKF, vector<pair<int, int>> &matches,
                      vector<cv::Point3f> &point_3d, vector<pair<int, int>> &match3d2d);
    cv::Point2f pixel2cam(cv::Point_<float> p, cv::Mat &K);

    void SendToTracking(KeyFrame* mCurrKF, local_map::Track_LocalMap::Response &resp);

    void InsertNewKeyFrame(KeyFrame* pKF);
    KeyFrame* GetNewKeyFrame();

    KeyFrame* GetKeyFrame(local_map::Track_LocalMap::Request &keyframe_msg);

public:

//    ros::Publisher pub_mappoint;

    KeyFrame* mInitKeyFrame;
//    KeyFrame* mLastKeyFrame;

    vector<KeyFrame*> mDataBaseKeyFrame;
    map<unsigned long, MapPoint*> mDataBaseMapPoint;

    list<KeyFrame*> mListNewKeyFrame;

    ORBVocabulary* mVocabulary;
//    KeyFrame* mCurrKeyFrame;
    float fx,fy,cx,cy;
    cv::Mat mk;

    unsigned long mMaxMapPointId;

};

#endif //LOCAL_MAP_LOCALMAP_H
