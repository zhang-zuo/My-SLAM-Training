//
// Created by zhangzuo on 22-8-8.
//

#ifndef SRC_TRACKING_H
#define SRC_TRACKING_H

#include "Frame.h"
#include "MapPoint.h"
#include "ros/ros.h"
#include <iostream>
#include "time.h"
#include <string>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "feature_tracking/Track_LocalMap.h"
#include "mutex"
#include <thread>
#include <nav_msgs/Path.h>
#include "Eigen/Geometry"



using namespace std;
//using namespace Eigen;

class Frame;


class yamlfile{
public:
    //下面是从配置文件读取的
    float fx,fy,cx,cy;
    float k1,k2,p1,p2;
    int nFeature;
    float scaleFactor;
    int nLevels;
    int iniThFAST;
    int minThFAST;
    cv::Mat mk;
    cv::Mat mDistCoef;
};


class Tracking{
public:
    Tracking(ros::NodeHandle &n);

    void process();

    void img_callback(const sensor_msgs::ImageConstPtr &img_msg);
    void localmap_callback(const feature_tracking::Track_LocalMap &localmap_msg);

    void MonoInitialization();

    int SearchForInit(vector<cv::DMatch> &good_matches);

    //这里的重载其实两个函数是一样的内容，只不过再track中使用这个函数时候没想好怎么改，因此直接重载了
    //以后可以将这个写成模板，应该比较合适一些
    void ComputeThreeMaxima(vector<int>* histo, const int L, int &ind1, int &ind2, int &ind3);
    void ComputeThreeMaxima(vector<pair<int, pair<int, int>>>* histo, const int L, int &ind1, int &ind2, int &ind3);

    bool Initialization(vector<cv::DMatch> &good_matches, cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &point_3d);

    float ComputeSToMapPointDepth(int q);

    int SearchForTrack(float th_x, float th_y, bool ComeKeyFrame);
    int DescriptorDistance(const cv::Mat &a, const cv::Mat &b);

    void SendKeyFrame();

    void Track();
    void LocalMapTrack();

    bool GetNewFrame(cv::Mat &img, double &imgtime);

public:

    sensor_msgs::ImagePtr msg_image;
    feature_tracking::Track_LocalMap msg_keyframe;

    list<pair<cv::Mat,double>> mListNewFrame;
    Frame mCurrentFrame;
    Frame mLastFrame;
    Frame mInitFrame;
    //这里的map实际上应该不会存很多，也就一两个，因为local_map很快会发送地图点过来，然后就remove了
    //使用map的好处，可以使用key值进行快速搜索
    map<unsigned long, Frame*> mapKeyFrame;
    Frame* mKeyFrame;

    std::vector<pair<int, int>> match_3d2d;

    cv::Mat mVelocity; //相对速度

    bool InitFinish = false;
    bool InitFrame_First = true;
    bool Init_send = false;
    bool FindKeyFrame = false;
    pair<bool, unsigned long> NewKeyFrame;

    int mCurrTrackMapPointSize;


};





#endif //SRC_TRACKING_H
