//
// Created by zhangzuo on 22-8-19.
//

#include "LocalMap.h"
#include "KeyFrame.h"
#include "opencv2/opencv.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/flann.hpp"
#include "opencv2/highgui/highgui.hpp"

std::mutex m;
const int HIST_LENGTH = 30;
LocalMapping::LocalMapping()
{}

LocalMapping::LocalMapping(ros::NodeHandle &n)
{
    //读取配置文件
    std::string ORBVoc_file;
    if(n.getParam("ORBVoc_file", ORBVoc_file))
    {
        ROS_DEBUG_STREAM("Loaded ORBVoc_file: " << ORBVoc_file);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load ORBVoc_file " << ORBVoc_file);
        n.shutdown();
    }

    mVocabulary = new ORBVocabulary();
    bool LoadVoc = mVocabulary->loadFromTextFile(ORBVoc_file);
    if(!LoadVoc)
    {
        ROS_ERROR_STREAM("Wrong to load Vocabulary!");
        exit(-1);
    }
    ROS_WARN_STREAM("Vocabulary load!");


}


//查找关键帧的共视帧,后面是数量
vector<KeyFrame*> LocalMapping::FindLocalCoviewing(KeyFrame *mCKeyFrame, int num)
{
//    KeyFrame* FindLastKeyFrame = *(mDataBaseKeyFrame.end()-1);
//    auto mCMapPoints = mCKeyFrame->GetmvpMapPoint();

//    for(map<unsigned long, pair<MapPoint*, int>>::iterator it = mCMapPoints.begin(); it != mCMapPoints.end(); it++)
    vector<KeyFrame*> CoviewKeyFrames;

    if(num > mCKeyFrame->mCoviewing.size())
        num = mCKeyFrame->mCoviewing.size();

    for(int i = 0; i < num; i++)
    {
        CoviewKeyFrames.push_back(mCKeyFrame->mCoviewing[i].first);
//        ROS_WARN_STREAM("CoviewKeyFrames-mid: " << mCKeyFrame->mCoviewing[i].first->mid <<
//                        "\nCoviewKeyFrames-des: \n" << mCKeyFrame->mCoviewing[i].first->mDescriptors.rowRange(0,4));
    }

    return CoviewKeyFrames;
}

//查找当前关键帧的共视关键帧，也就是局部地图
vector<KeyFrame*> LocalMapping::FindLocalKeyFrame(KeyFrame *mCKeyFrame, int weight) {

    vector<KeyFrame*> CoviewKeyFrames;

    for(int i = 0; i < mCKeyFrame->mCoviewing.size(); i++)
    {
        if(mCKeyFrame->mCoviewing[i].second >= weight)
            CoviewKeyFrames.push_back(mCKeyFrame->mCoviewing[i].first);
//        ROS_WARN_STREAM("CoviewKeyFrames-mid: " << mCKeyFrame->mCoviewing[i].first->mid << "\tint: " << mCKeyFrame->mCoviewing[i].second);
    }

    return CoviewKeyFrames;
}



//此函数根据VINS中对关键帧的筛选而作，但是这个阈值设定的其实没有多大的约束力，因为这个阈值小了点
bool LocalMapping::CheckKeyFrameParallax(KeyFrame *mCurrKF, KeyFrame *mCoviewKF)
{
    auto mCurrMapPoints = mCurrKF->GetmvpMapPoint();
    auto mCoviewMapPoints = mCoviewKF->GetmvpMapPoint();

    double sum_parallax = 0;
    double average_parallax;
    int num = 0;
    for(auto mCurrMP = mCurrMapPoints.begin(); mCurrMP != mCurrMapPoints.end(); mCurrMP++)
    {
        unsigned long mCurrId = mCurrMP->first;
        map<unsigned long, pair<MapPoint*, int>>::iterator mCoviewPos = mCoviewMapPoints.find(mCurrId);

        cv::KeyPoint mCurrKP = mCurrKF->mKeys[mCurrMP->second.second];
        cv::KeyPoint mCoviewKP = mCoviewKF->mKeys[mCoviewPos->second.second];

        Eigen::Vector2d mCurrUV(mCurrKP.pt.x, mCurrKP.pt.y);
        Eigen::Vector2d mCoviewUV(mCoviewKP.pt.x, mCoviewKP.pt.y);
        double parallax = (mCurrUV - mCoviewUV).norm();
        sum_parallax += parallax;
        num++;
    }
    average_parallax = sum_parallax / num;


    ROS_WARN_STREAM("mnid: " << mCoviewKF->mid << "\taverage_parallax: " << average_parallax);
    //两帧之间视差角太小了，三角化出来的点不准确
    if(average_parallax * fx > 10)
        return true;

    return false;
}

void LocalMapping::SearchForLocalMap(KeyFrame *mCurrKF, KeyFrame *mCoviewKF, cv::Mat &F21, vector<pair<int, int>> &matchIndex)
{
//    ROS_WARN_STREAM("matcher!!");
    //这里经过测试，使用opencv的flann去匹配ORB特征点的话，只能使用LSH方法进行描述子距离计算
    //但是LSH方法计算描述子距离非常慢，这个速度比BF方法都要慢
    //因此最后只能使用ORBSLAM2的词袋加速匹配
    //这里的1帧指的是共视关键帧，因为这个关键帧中没有匹配的特征点少一点

//    vector<vector<cv::DMatch>> matches;
//    cv::Ptr<cv::FlannBasedMatcher> matcher = cv::FlannBasedMatcher::create();
//    cv::FlannBasedMatcher matcher = cv::FlannBasedMatcher(cv::makePtr<cv::flann::LshIndexParams>(12,20,2));
//    cv::FlannBasedMatcher matcher(cv::makePtr<cv::flann::LshIndexParams>(12,20,2));
//    matcher->knnMatch(mCurrKF->mDescriptors, mCoviewKF->mDescriptors, matches, 2);
//    mCurrKF->mDescriptors.convertTo(mCurrKF->mDescriptors, CV_32F);
//    mCoviewKF->mDescriptors.convertTo(mCoviewKF->mDescriptors, CV_32F);
//    matcher.knnMatch(mCurrKF->mDescriptors, mCoviewKF->mDescriptors, matches, 2);

    //1.将360度建立成30个直方图bin
    vector<pair<int, pair<int, int>>> rotHist[HIST_LENGTH];
    //2.初步定义每个直方图空间位500
    for(int i1 = 0; i1 < HIST_LENGTH; i1++)
        rotHist[i1].reserve(150);

    const float factor = HIST_LENGTH/360.0f;
    int HIGH_DIS = 40;

    const DBoW2::FeatureVector &vFeatVec1 = mCoviewKF->mFeatVec;
    const DBoW2::FeatureVector &vFeatVec2 = mCurrKF->mFeatVec;

    DBoW2::FeatureVector::const_iterator f1it = vFeatVec1.begin();
    DBoW2::FeatureVector::const_iterator f2it = vFeatVec2.begin();
    DBoW2::FeatureVector::const_iterator f1end = vFeatVec1.end();
    DBoW2::FeatureVector::const_iterator f2end = vFeatVec2.end();

    //遍历pKF1和pKF2中的node节点
    while(f1it!=f1end && f2it!=f2end)
    {
        // 如果f1it和f2it属于同一个node节点才会进行匹配，这就是BoW加速匹配原理
        if(f1it->first == f2it->first)
        {
            //遍历属于同一node节点(id：f1it->first)下的所有特征点
            for(size_t i1=0, iend1=f1it->second.size(); i1<iend1; i1++)
            {
                //获取pKF1中属于该node节点的所有特征点索引
                const int idx1 = f1it->second[i1];

                if(mCoviewKF->KeyPointisMapPoint[idx1])
                    continue;

                cv::KeyPoint mCoviewKP = mCoviewKF->mKeys[idx1];
                cv::Mat mCoviewDes = mCoviewKF->mDescriptors.row(idx1);

                int bestDis = 256;
                int bestDis2 = 256;
                int bestDis_idx = -1;
                //遍历该node节点下(f2it->first)对应KF2中的所有特征点
                for(size_t i2=0, iend2=f2it->second.size(); i2<iend2; i2++)
                {
                    // 获取pKF2中属于该node节点的所有特征点索引
                    int idx2 = f2it->second[i2];

                    if(mCurrKF->KeyPointisMapPoint[idx2])
                        continue;

//                    cv::KeyPoint mCurrKP = mCurrKF->mKeys[idx2];
                    cv::Mat mCurrDes = mCurrKF->mDescriptors.row(idx2);

                    int distance = DescriptorDistance(mCoviewDes,mCurrDes);

                    if(distance > 50)
                        continue;

//                    if(!CheckEpipolarLine(mCoviewKP, mCurrKP, F21))
//                        continue;

                    if(distance < bestDis)
                    {
                        bestDis2 = bestDis;
                        bestDis = distance;
                        bestDis_idx = idx2;
                    }
                    else if(distance < bestDis2)
                    {
                        bestDis2 = distance;
                    }
                }

                if(bestDis > (float)bestDis2*0.8)
                    continue;

                cv::KeyPoint mCurrKP = mCurrKF->mKeys[bestDis_idx];
                if(!CheckEpipolarLine(mCoviewKP, mCurrKP, F21))
                    continue;

                //放入旋转直方图
                if(bestDis < HIGH_DIS)
                {
                    cv::KeyPoint mCurrKey = mCurrKF->mKeys[bestDis_idx];

                    float rot = mCoviewKP.angle - mCurrKey.angle;
                    if(rot < 0)
                        rot += 360;
                    int bin = round(rot*factor);
                    if(bin == HIST_LENGTH)
                        bin = 0;
                    auto LC_idx = make_pair(idx1, make_pair(bestDis_idx, bestDis));
                    rotHist[bin].push_back(LC_idx);

//                    ROS_WARN_STREAM("kp1-idx: " << idx1 << "\tkp2-idx: " << bestDis_idx << "\tdis: " << bestDis);
//                    ROS_WARN_STREAM("kp1-des: " << mCurrKF->mDescriptors.row(idx1));
//                    ROS_WARN_STREAM("kp2-des: " << mCoviewKF->mDescriptors.row(bestDis_idx));
                }
            }

            f1it++;
            f2it++;
        }
        else if(f1it->first < f2it->first)
        {
            f1it = vFeatVec1.lower_bound(f2it->first);
        }
        else
        {
            f2it = vFeatVec2.lower_bound(f1it->first);
        }
    }

    map<int, pair<int, int>> FilterKeyPoint;   //mCurrIdx, mCoviewIdx, dis
    int ind1=-1;
    int ind2=-1;
    int ind3=-1;
    // 筛选出在旋转角度差落在在直方图区间内数量最多的前三个bin的索引
    ComputeThreeMaxima(rotHist,HIST_LENGTH,ind1,ind2,ind3);

    for(int i2 = 0; i2 < HIST_LENGTH; i2++)
    {
        if(i2 == ind1 || i2 == ind2 || i2 == ind3)
        {
            for(size_t j = 0; j < rotHist[i2].size(); j++)
            {
                int mCoviewIdx = rotHist[i2][j].first;
                int mCurrIdx = rotHist[i2][j].second.first;
                int mDistance = rotHist[i2][j].second.second;

                if(FilterKeyPoint.empty())
                {
                    FilterKeyPoint.insert(make_pair(mCurrIdx, make_pair(mCoviewIdx, mDistance)));
                    continue;
                }

                auto KPpos = FilterKeyPoint.find(mCurrIdx);
                if(KPpos != FilterKeyPoint.end())
                {
                    if(mDistance <= KPpos->second.second)
                        FilterKeyPoint[mCurrIdx] = make_pair(mCoviewIdx, mDistance);
                }
                else
                    FilterKeyPoint.insert(make_pair(mCurrIdx, make_pair(mCoviewIdx, mDistance)));
//                matchIndex.push_back(make_pair(rotHist[i2][j].first,rotHist[i2][j].second));
            }
        }
//        ROS_INFO_STREAM("rotHist[" << i2 << "].size: " << rotHist[i2].size());
    }

    for(auto it = FilterKeyPoint.begin(); it != FilterKeyPoint.end(); it++)
        matchIndex.push_back(make_pair(it->second.first, it->first));

//    for(auto matches : matchIndex)
//    {
//        ROS_WARN_STREAM("1-idx: " << matches.first << "\t1-key: " << mCoviewKF->mKeys[matches.first].pt);
//        ROS_WARN_STREAM("1-des: " << mCoviewKF->mDescriptors.row(matches.first));
//        ROS_WARN_STREAM("2-idx: " << matches.second << "\t2-key: " << mCurrKF->mKeys[matches.second].pt);
//        ROS_WARN_STREAM("2-des: " << mCurrKF->mDescriptors.row(matches.second));
//    }

//    ROS_INFO_STREAM("local_map => Search For LocalMap matche size: " << matchIndex.size());


    //显示匹配结果
//    float i = 0.0;
//    vector<cv::DMatch> matches_test;
//    for(auto midx : matchIndex)
//    {
//        cv::DMatch match_idx = cv::DMatch(midx.first,midx.second,i++);
//        matches_test.push_back(match_idx);
//    }
//
//    cv::Mat img_in;
//    cv::drawMatches(mCoviewKF->mKeyFrameImage,mCoviewKF->mKeys,mCurrKF->mKeyFrameImage,mCurrKF->mKeys,matches_test,img_in);
//    cv::imshow("img_in", img_in);
//    cv::waitKey(0);


}


// http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
int LocalMapping::DescriptorDistance(const cv::Mat &a, const cv::Mat &b)
{
    const int *pa = a.ptr<int32_t>();
    const int *pb = b.ptr<int32_t>();

    int dist=0;

    // 8*32=256bit

    for(int i=0; i<8; i++, pa++, pb++)
    {
        unsigned  int v = *pa ^ *pb;        // 相等为0,不等为1
        // 下面的操作就是计算其中bit为1的个数了,这个操作看上面的链接就好
        // 其实我觉得也还阔以直接使用8bit的查找表,然后做32次寻址操作就完成了;不过缺点是没有利用好CPU的字长
        v = v - ((v >> 1) & 0x55555555);
        v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
        dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
    }

    return dist;
}

void LocalMapping::ComputeF21(KeyFrame *mCoviewKF, KeyFrame *mCurrKF, cv::Mat &F21)
{
    //按照slam十四讲中，1应该是共视帧，2是当前帧
    cv::Mat T1w = mCoviewKF->GetPose();
    cv::Mat T2w = mCurrKF->GetPose();

    cv::Mat R1w = T1w.rowRange(0,3).colRange(0,3);
    cv::Mat t1w = T1w.rowRange(0,3).col(3);
    cv::Mat R2w = T2w.rowRange(0,3).colRange(0,3);
    cv::Mat t2w = T2w.rowRange(0,3).col(3);

    cv::Mat R21 = R2w * R1w.t();
    cv::Mat t21 = -R2w*R1w.t()*t1w + t2w;

    cv::Mat t21x = (cv::Mat_<float>(3,3) <<
                    0,                      -t21.at<float>(2),  t21.at<float>(1),
                    t21.at<float>(2),    0,                     -t21.at<float>(0),
                    -t21.at<float>(1),   t21.at<float>(0),   0                      );

    F21 = mk.t().inv() * t21x * R21 * mk.inv();
//    ROS_WARN_STREAM("T1w: " << T1w.reshape(0,1));
//    ROS_WARN_STREAM("T2w: " << T2w.reshape(0,1));
//    ROS_WARN_STREAM("F21: " << F21.reshape(0,1));
}

bool LocalMapping::CheckEpipolarLine(cv::KeyPoint &kp1, cv::KeyPoint &kp2, cv::Mat &F21)
{
    cv::Mat kp1c = (cv::Mat_<float>(3,1) << kp1.pt.x, kp1.pt.y, 1);
    cv::Mat line2 = F21 * kp1c;

    float ll = line2.at<float>(0)*kp2.pt.x + line2.at<float>(1)*kp2.pt.y + line2.at<float>(2);
    float distance_2 = ll * ll /(line2.at<float>(0)*line2.at<float>(0) + line2.at<float>(1)*line2.at<float>(1));

    //这块1.44不严谨
    const double sigma2 = 1 / pow(1.44, kp2.octave);
    return distance_2 < 3.84;
}

void LocalMapping::ComputeThreeMaxima(vector<pair<int, pair<int, int>>>* histo, const int L, int &ind1, int &ind2, int &ind3)
{
    int max1=0;
    int max2=0;
    int max3=0;

    for(int i=0; i<L; i++)
    {
        const int s = histo[i].size();
        if(s>max1)
        {
            max3=max2;
            max2=max1;
            max1=s;
            ind3=ind2;
            ind2=ind1;
            ind1=i;
        }
        else if(s>max2)
        {
            max3=max2;
            max2=s;
            ind3=ind2;
            ind2=i;
        }
        else if(s>max3)
        {
            max3=s;
            ind3=i;
        }
    }

    // 如果差距太大了,说明次优的非常不好,这里就索性放弃了,都置为-1
    if(max2<0.1f*(float)max1)
    {
        ind2=-1;
        ind3=-1;
    }
    else if(max3<0.1f*(float)max1)
    {
        ind3=-1;
    }
}

//下面这段没使用，因为忙活半天，结果看opencv的源码时发现，recoverPose函数已经三角化了地图点,只是没有合法性判断
//而且还留了接口出来，我都怀疑ORBSLAM2和OpenCV是谁参考的谁，有点像啊。。。。。
void LocalMapping::Trangulation(KeyFrame *mCurrKF, KeyFrame *mCoviewKF, vector<pair<int, int>> &matches,
                                vector<cv::Point3f> &point_3d, vector<pair<int, int>> &match3d2d)
{
    //使用opencv的函数进行三角化
    vector<cv::Point2f> pts_1, pts_2;
    cv::Mat pts_4;
    cv::Mat T1w = mCoviewKF->GetPose();
    cv::Mat T2w = mCurrKF->GetPose();

//    cv::Mat T1 = (cv::Mat_<float>(3,4) <<
//            T1w.at<float>(0,0), T1w.at<float>(0,1), T1w.at<float>(0,2), T1w.at<float>(0,3),
//            T1w.at<float>(1,0), T1w.at<float>(1,1), T1w.at<float>(1,2), T1w.at<float>(1,3),
//            T1w.at<float>(2,0), T1w.at<float>(2,1), T1w.at<float>(2,2), T1w.at<float>(2,3));
//    cv::Mat T2 = (cv::Mat_<float>(3,4) <<
//            T2w.at<float>(0,0), T2w.at<float>(0,1), T2w.at<float>(0,2), T2w.at<float>(0,3),
//            T2w.at<float>(1,0), T2w.at<float>(1,1), T2w.at<float>(1,2), T2w.at<float>(1,3),
//            T2w.at<float>(2,0), T2w.at<float>(2,1), T2w.at<float>(2,2), T2w.at<float>(2,3));
    cv::Mat K = (cv::Mat_<float>(3,3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);

    cv::Mat T1 = mCoviewKF->GetPose().rowRange(0,3);
    cv::Mat T2 = mCurrKF->GetPose().rowRange(0,3);

    cv::Mat R1 = T1.rowRange(0,3).colRange(0,3);
    cv::Mat t1 = T1.rowRange(0,3).col(3);
    cv::Mat R2 = T2.rowRange(0,3).colRange(0,3);
    cv::Mat t2 = T2.rowRange(0,3).col(3);

//    ROS_WARN_STREAM("T1: " << T1.reshape(0,1));
//    ROS_WARN_STREAM("T2: " << T2.reshape(0,1));

    for(int i = 0; i < matches.size(); i++)
    {
        pts_1.push_back(pixel2cam(mCoviewKF->mKeys[matches[i].first].pt, K));
        pts_2.push_back(pixel2cam(mCurrKF->mKeys[matches[i].second].pt, K));
//        ROS_WARN_STREAM("pt1: " << pixel2cam(mCoviewKF->mKeys[matches[i].first].pt, K));
//        ROS_WARN_STREAM("pt2: " << pixel2cam(mCurrKF->mKeys[matches[i].second].pt, K));
    }

    cv::triangulatePoints(T1,T2,pts_1,pts_2,pts_4);

    pts_4.convertTo(pts_4,CV_32F);
    for(int i1 = 0; i1 < pts_4.cols; i1++)
    {
        cv::Mat x = pts_4.col(i1);
        x /= x.at<float>(3,0);
        cv::Mat p3d = x.rowRange(0,3);

        //转换到各自坐标系的坐标
        cv::Mat xyz1 = R1 * p3d + t1;
        cv::Mat xyz2 = R2 * p3d + t2;

        if(xyz1.at<float>(2) <= 0 || xyz2.at<float>(2) <= 0)
            continue;

        float sigma1 = pow(1.44,mCoviewKF->mKeys[matches[i1].first].octave);
        float sigma2 = pow(1.44,mCurrKF->mKeys[matches[i1].second].octave);
        cv::Point2f uv1 = mCoviewKF->mKeys[matches[i1].first].pt;
        cv::Point2f uv2 = mCurrKF->mKeys[matches[i1].second].pt;

        float featu1 = fx*xyz1.at<float>(0)/xyz1.at<float>(2)+cx;
        float featv1 = fy*xyz1.at<float>(1)/xyz1.at<float>(2)+cy;
        float errX1 = featu1 - uv1.x;
        float errY1 = featv1 - uv1.y;
        if(errX1*errX1+errY1*errY1 > 5.99*sigma1)
            continue;

        float featu2 = fx*xyz2.at<float>(0)/xyz2.at<float>(2)+cx;
        float featv2 = fy*xyz2.at<float>(1)/xyz2.at<float>(2)+cy;
        float errX2 = featu2 - uv2.x;
        float errY2 = featv2 - uv2.y;
        if(errX2*errX2+errY2*errY2 > 5.99*sigma2)
            continue;


        cv::Point3f p(
                x.at<float>(0,0),
                x.at<float>(1,0),
                x.at<float>(2,0)
        );
        point_3d.push_back(p);
        match3d2d.push_back(make_pair(matches[i1].first,matches[i1].second));
//        ROS_INFO_STREAM("p: " << p);
    }

//    ROS_INFO_STREAM("local_map => Trangulation MapPoints size: " << point_3d.size());
    //对三角化的地图点进行合法性检查
}

cv::Point2f LocalMapping::pixel2cam(cv::Point_<float> p, cv::Mat &K) {
    //ROS_INFO_STREAM("p.x: " << p.x << " K1: " << K.at<float>(0,2) << " K2: " << K.at<float>(0,0));
    return cv::Point2f
            (
                    (p.x - K.at<float>(0,2)) / K.at<float>(0,0),
                    (p.y - K.at<float>(1,2)) / K.at<float>(1,1)
            );
}

void LocalMapping::InsertNewKeyFrame(KeyFrame *pKF)
{
    //锁
    m.lock();
    mListNewKeyFrame.push_back(pKF);
    m.unlock();
}

KeyFrame *LocalMapping::GetNewKeyFrame()
{
    //锁
    m.lock();
    if(mListNewKeyFrame.empty())
    {
//        ROS_WARN_STREAM("NULL");
        m.unlock();
        return NULL;
    }


    KeyFrame* mCKF = mListNewKeyFrame.front();
    mListNewKeyFrame.pop_front();
    m.unlock();
    return mCKF;
}