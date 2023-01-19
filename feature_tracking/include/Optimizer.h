//
// Created by zhangzuo on 22-8-16.
//

#ifndef SRC_OPTIMIZER_H
#define SRC_OPTIMIZER_H

#include "Tracking.h"
#include "MapPoint.h"
#include "Frame.h"
#include "Eigen/Core"
#include "sophus/se3.hpp"

void GlobalBAInit(vector<Frame *> FrameNum, vector<pair<MapPoint*, int>> &p3d);
void TrackingBA(Frame *CurrFrame, vector<pair<MapPoint*, int>> &CurrMapPoint);


#endif //SRC_OPTIMIZER_H
