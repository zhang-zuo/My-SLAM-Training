//
// Created by zhangzuo on 22-8-9.
//

#ifndef SRC_FRAME_H
#define SRC_FRAME_H

#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
using namespace std;


class Feature;
class MapPoint;
class feature_tracking_node;
struct yamlFile;


class Frame{
public:
    Frame();
    Frame(const Frame &frame);
    Frame(cv::Mat &img, double &time_stamp);

    void ExtractORB(cv::Mat &imgGray);

    void undistortKeyPoint();

    void SetPose(cv::Mat Tcw){
        mTcw = Tcw.clone();
    }

    cv::Mat GetPose(){
        return mTcw.clone();
    }

    void AddMapPoint(MapPoint *pMP, const int idx);

    vector<pair<MapPoint*, int>> GetmvpMapPoint();

    void EarseAllMapPoint();
    void DeleteAllMapPoint();

public:

    static long unsigned int mnId;

    unsigned long mid = 0;
    unsigned long mkeyframe_id = 0;
    bool is_keyframe = false;
    double mTimeStamp;

    cv::Mat mTcw;

    std::vector<cv::KeyPoint> mKeys;  //未校正
    std::vector<cv::KeyPoint> unKeys; //校正后的特征点集合
    cv::Mat mDescriptors;

    vector<bool> KeyPointisMapPoint;

    cv::Mat mFrameImg;

    vector<pair<MapPoint*, int>> mvpMapPoint;

    std::vector<Feature> mfeature2d;

    int mReferKeyFrameId;

    vector<int> mCoviewFrameId;
};

//Feature类还需要存在吗？特征点都是用cv::KeyPoint存放的
class Feature{
public:

    Feature();

public:

    unsigned long mFrameId;
    cv::KeyPoint mkeyPoint;

//    std::weak_ptr<Frame*> mframe;
//    std::weak_ptr<MapPoint*> mapPoint;

    bool is_outlier = false;


};

// 分配四叉树时用到的结点类型
class ExtractorNode
{
public:
    /** @brief 构造函数 */
    ExtractorNode():bNoMore(false){}

    /**
     * @brief 在八叉树分配特征点的过程中，实现一个节点分裂为4个节点的操作
     *
     * @param[out] n1   分裂的节点1
     * @param[out] n2   分裂的节点2
     * @param[out] n3   分裂的节点3
     * @param[out] n4   分裂的节点4
     */
    void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);

    ///保存有当前节点的特征点
    std::vector<cv::KeyPoint> vKeys;
    ///当前节点所对应的图像坐标边界
    cv::Point2i UL, UR, BL, BR;
    //存储提取器节点的列表（其实就是双向链表）的一个迭代器,可以参考[http://www.runoob.com/cplusplus/cpp-overloading.html]
    //这个迭代器提供了访问总节点列表的方式，需要结合cpp文件进行分析
    std::list<ExtractorNode>::iterator lit;

    ///如果节点中只有一个特征点的话，说明这个节点不能够再进行分裂了，这个标志置位
    //这个节点中如果没有特征点的话，这个节点就直接被删除了
    bool bNoMore;
};



#endif //SRC_FRAME_H
