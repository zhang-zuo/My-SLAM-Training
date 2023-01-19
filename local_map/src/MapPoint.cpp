//
// Created by zhangzuo on 22-8-19.
//

#include "MapPoint.h"


MapPoint::MapPoint()
{}

MapPoint::MapPoint(cv::Mat pos)
{
    mWorldPos = pos;
    pid++;
    mid = pid;
}

MapPoint::MapPoint(cv::Mat pos, unsigned long id){
    mWorldPos = pos.clone();
    mid = id;
    pid = id;
//    ROS_WARN_STREAM("pid: " << pid);
}


void MapPoint::SetWorldPos(cv::Mat worldpos) {
    mWorldPos = worldpos.clone();
}

cv::Mat MapPoint::GetWorldPos() {
    return mWorldPos.clone();
}

void MapPoint::AddObservation(KeyFrame *mKF, int idx)
{
    if(observation.count(mKF))
        return;

    observation[mKF] = idx;
}

map<KeyFrame*, int> MapPoint::GetObservation()
{
    //锁
    return observation;
}

void MapPoint::EarseOneObservation(KeyFrame *KF)
{
    if(observation.find(KF) == observation.end())
        return;

    observation.erase(KF);
}

