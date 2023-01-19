//
// Created by zhangzuo on 22-8-19.
//

#ifndef LOCAL_MAP_KEYFRAME_H
#define LOCAL_MAP_KEYFRAME_H

#include <iostream>
#include "LocalMap.h"
#include "MapPoint.h"
#include <opencv2/core/core.hpp>
#include "map"
#include "set"
#include "DBoW2/DBoW2/BowVector.h"
#include "DBoW2/DBoW2/FeatureVector.h"
#include "DBoW2/DBoW2/FORB.h"
#include "DBoW2/DBoW2/TemplatedVocabulary.h"

using namespace std;
typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB> ORBVocabulary;

class MapPoint;

class KeyFrame{
public:
    KeyFrame();
    KeyFrame(ORBVocabulary* voc);

    void SetPose(cv::Mat Tcw);
    cv::Mat GetPose();

    void AddMapPoint(unsigned long mpid, MapPoint *pMP, const int idx);

    map<unsigned long, pair<MapPoint*, int>> GetmvpMapPoint();

    void UpdateCoviewing();
    void ComputeBoW();

    void EarseAllMapPoint();
    void EarseOneMapPoint(unsigned long id);
    void DeleteAllMapPoint();

public:
    static long unsigned int mnId;
    unsigned long mid = 0;
    unsigned long frameId = 0;
    double mTimeStamp;

    cv::Mat mTcw;
    vector<cv::KeyPoint> mKeys;
    cv::Mat mDescriptors;
    //这个显示图像部分是测试使用，实际运行过程中不会使用
    cv::Mat mKeyFrameImage;

    unsigned long ParkInOptKeyFrameId;
    //共视关系，指的是对于当前关键帧中，与其他关键帧之间的共视数量
    //关键帧，权重（共视数量）
    vector<pair<KeyFrame*, int>> mCoviewing;

    //判断这个特征点是否是地图点，为了便于搜索，使用了set容器
    //int存放的是MapPoint再mKeys中的索引
    vector<bool> KeyPointisMapPoint;

    //BoW向量其实没有使用，只不过转换的时候需要用
    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;

    ORBVocabulary* mKFVocabulary;

    map<unsigned long, pair<MapPoint*, int>> mvpMapPoint;

    vector<int> mCoviewFrameId;
};

#endif //LOCAL_MAP_KEYFRAME_H
