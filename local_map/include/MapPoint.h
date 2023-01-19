//
// Created by zhangzuo on 22-8-19.
//

#ifndef LOCAL_MAP_MAPPOINT_H
#define LOCAL_MAP_MAPPOINT_H

#include "LocalMap.h"
#include "KeyFrame.h"
#include <opencv2/core/core.hpp>
#include "map"

using namespace std;

static long unsigned int pid;

class KeyFrame;

class MapPoint{
public:
    MapPoint();
    MapPoint(cv::Mat pos);
    MapPoint(cv::Mat pos, unsigned long id);

    void SetWorldPos(cv::Mat worldpos);
    cv::Mat GetWorldPos();
    void AddObservation(KeyFrame* mKF, int idx);
    map<KeyFrame*, int> GetObservation();
    void EarseOneObservation(KeyFrame* KF);

public:

    unsigned long mid = 0;
    bool is_outlier = false;

    //观测关系，关键帧，该地图点在关键帧mkeys中的索引
    map<KeyFrame*, int> observation;

    unsigned long mPartOptKeyFrameId;

    cv::Mat mWorldPos;
};

#endif //LOCAL_MAP_MAPPOINT_H
