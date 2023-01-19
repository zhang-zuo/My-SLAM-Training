//
// Created by zhangzuo on 22-8-19.
//
#include "KeyFrame.h"
#include "LocalMap.h"

KeyFrame::KeyFrame()
{}

KeyFrame::KeyFrame(ORBVocabulary* voc)
{
    mKFVocabulary = voc;
}

void KeyFrame::SetPose(cv::Mat Tcw)
{
    mTcw = Tcw.clone();
}

cv::Mat KeyFrame::GetPose()
{
    return mTcw.clone();
}

void KeyFrame::AddMapPoint(unsigned long mpid, MapPoint *pMP, const int idx)
{
    //加上local_map之后，需要添加锁
    auto MapPoint_idx = make_pair(mpid, make_pair(pMP, idx));
    mvpMapPoint.insert(MapPoint_idx);
}

map<unsigned long, pair<MapPoint*, int>> KeyFrame::GetmvpMapPoint()
{
    return mvpMapPoint;
}

void KeyFrame::UpdateCoviewing()
{
    map<KeyFrame*, int> Counter;
    auto mCurrMapPoint = this->GetmvpMapPoint();

    for(auto it0 = mCurrMapPoint.begin(); it0 != mCurrMapPoint.end(); it0++)
    {
        MapPoint* mCMP =  it0->second.first;
        map<KeyFrame*, int> mCurrObservation = mCMP->GetObservation();
        for(auto it1 = mCurrObservation.begin(); it1 != mCurrObservation.end(); it1++)
        {
            KeyFrame* pKF = it1->first;
            if(pKF->mid == mid)
                continue;

            Counter[pKF]++;
        }
    }

    //把共视关键帧和权重放入到mCoviewing
    //如果要设置这个连接关系权重数量的限制，就在这里添加
    this->mCoviewing.clear();
    for(map<KeyFrame* ,int>::iterator ic = Counter.begin(); ic != Counter.end(); ic++)
        this->mCoviewing.push_back(make_pair(ic->first,ic->second));

    //排序，mCoviewing中按照权重从大到小
    sort(this->mCoviewing.begin(),this->mCoviewing.end(),
         [](const pair<KeyFrame*, int> &kpa, const pair<KeyFrame*, int> &kpb){
        return kpa.second > kpb.second;});

//    for(int i = 0; i < this->mCoviewing.size(); i++)
//    {
//        ROS_INFO_STREAM("local_map => UpdateCoviewing KF: " << this->mCoviewing[i].first->mid
//                        << "\tint: " << this->mCoviewing[i].second
////                        << "\nKF-des: \n" << this->mCoviewing[i].first->mDescriptors.rowRange(0,4)
//                        );
//    }
}

void KeyFrame::ComputeBoW()
{
    if(mBowVec.empty() || mFeatVec.empty())
    {
        vector<cv::Mat> mCurrDes;
        mCurrDes.reserve(this->mDescriptors.rows);

        for(int i = 0; i < this->mDescriptors.rows; i++)
            mCurrDes.push_back(this->mDescriptors.row(i));

//        LocalMapping::mVocabulary->transform(mCurrDes, mBowVec, mFeatVec, 4);
        mKFVocabulary->transform(mCurrDes, mBowVec, mFeatVec, 4);
    }
}

void KeyFrame::EarseAllMapPoint()
{
    //需要锁
    mvpMapPoint.clear();
}

void KeyFrame::EarseOneMapPoint(unsigned long id)
{
    //锁
    mvpMapPoint.erase(id);
}

void KeyFrame::DeleteAllMapPoint()
{
    //锁
    for(auto it = mvpMapPoint.begin(); it != mvpMapPoint.end(); it++)
    {
        delete it->second.first;
        ROS_WARN_STREAM("delete");
    }
}