//
// Created by zhangzuo on 22-8-9.
//

#ifndef SRC_MAPPOINT_H
#define SRC_MAPPOINT_H

#include <memory>
#include "Tracking.h"

class Feature;
class Frame;

class MapPoint{
public:

    MapPoint();
    MapPoint(cv::Mat &pos, unsigned long id);

    void SetWorldPos(cv::Mat worldpos){
        mWorldPos = worldpos.clone();
    }

    cv::Mat GetWorldPos(){
        return mWorldPos.clone();
    }

public:

    static long unsigned int mnid;
    unsigned long mid = 0;
    bool is_outlier = false;
    bool pflag = false;          //初始化过程中的标志位
//    list<weak_ptr<Feature>> mobservation;

    cv::Mat mWorldPos;


};


#endif //SRC_MAPPOINT_H
