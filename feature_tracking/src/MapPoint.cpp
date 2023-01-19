
#include "MapPoint.h"


long unsigned int MapPoint::mnid = 0;
MapPoint::MapPoint()
{}


MapPoint::MapPoint(cv::Mat &pos, unsigned long id) {
    mid = id;
    mWorldPos = pos.clone();
}