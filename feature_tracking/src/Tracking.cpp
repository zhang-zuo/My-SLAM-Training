//
// Created by zhangzuo on 22-8-10.
//
#include "Tracking.h"
#include "MapPoint.h"
#include "Optimizer.h"

extern yamlfile yamlFile;
const int HIST_LENGTH = 30;
extern std::mutex m_frame;
vector<pair<int, int>> MapPoint_iidx;
//void Tracking::ComputeThreeMaxima(vector<int>* histo, const int L, int &ind1, int &ind2, int &ind3);

void Tracking::MonoInitialization() {

    if(InitFrame_First)
    {
        //进来的第一帧，就是初始化两帧中的第一帧
        if(mCurrentFrame.unKeys.size() > 100){
            ROS_INFO_STREAM("InitFrame KeyPoint number: " << mCurrentFrame.unKeys.size());
            mInitFrame = Frame(mCurrentFrame);
            mLastFrame = Frame(mCurrentFrame);
            InitFrame_First = false;
            return;
        }
    }
    else
    {
        //初始化只在连续两帧特征点数大于100时才开始
        if(mCurrentFrame.unKeys.size() < 100){
            ROS_WARN_STREAM("SecondFrame KeyPoint too Less: " << mCurrentFrame.unKeys.size());
            InitFrame_First = true;
            return;
        }

        //将初始两帧进行匹配
        vector<cv::DMatch> mnInitMatches;
        int nmatches = 0;
        nmatches = SearchForInit(mnInitMatches);

        if(nmatches < 100){
            ROS_WARN_STREAM("nmatches too Less: " << mCurrentFrame.unKeys.size());
            InitFrame_First = true;
            return;
        }

        //开始进行初始化，主要就是根据匹配点求解Ｆ矩阵，然后分解得到Ｒｔ，后三角化得到初始地图点
        cv::Mat R21;
        cv::Mat t21;
        vector<cv::Point3f> point3d;

        if(Initialization(mnInitMatches,R21,t21,point3d))
        {

            ROS_INFO_STREAM("mInit-mnid: " << mInitFrame.mid);
            ROS_INFO_STREAM("mCurr-mnid: " << mCurrentFrame.mid);

            mInitFrame.SetPose(cv::Mat::eye(4,4,CV_32F));
            cv::Mat T21(4,4,CV_32F);
            R21.copyTo(T21.rowRange(0,3).colRange(0,3));
            t21.copyTo(T21.rowRange(0,3).col(3));
            mCurrentFrame.SetPose(T21);

            mInitFrame.is_keyframe = true;
            mCurrentFrame.is_keyframe = true;

            vector<Frame *> mFrame;
            mFrame.push_back(&mInitFrame);
            mFrame.push_back(&mCurrentFrame);

            float pp_x = 0.0;
            for(int i = 0; i < point3d.size(); i++){

                cv::Mat pp = (cv::Mat_<float>(3,1) << point3d[i].x,point3d[i].y,point3d[i].z );
                MapPoint* pMP = new MapPoint(pp,i);

                mInitFrame.AddMapPoint(pMP,match_3d2d[i].first);
                mCurrentFrame.AddMapPoint(pMP,match_3d2d[i].second);
                pp_x += point3d[i].x;
//                ROS_INFO_STREAM("p3d: " << pp.reshape(0,1)
//                                        << "\tp2d: "
//                                        << " mInit:" << mInitFrame.mKeys[match_3d2d[i].first].pt
//                                        << " \tmCurr:" << mCurrentFrame.mKeys[match_3d2d[i].second].pt
//                                        << " \tIni-idx: " << match_3d2d[i].first
//                                        << " \tCur-idx: " << match_3d2d[i].second);
            }

//            ROS_INFO_STREAM("point3d: " << point3d.size());
//            ROS_INFO_STREAM("pp_x: " << pp_x);
//            ROS_INFO_STREAM("pp_12: " << point3d[12].x);
//            ROS_INFO_STREAM("mCurrentFrame.mvpMapPoint.size(): " << mCurrentFrame.mvpMapPoint.size());
//            ROS_INFO_STREAM("mInitFrame.mvpMapPoint.size(): " << mInitFrame.mvpMapPoint.size());

            //g2o优化
            GlobalBAInit(mFrame,mInitFrame.mvpMapPoint);

            //此时两个关键帧的位姿已经优化了
            //但是地图点只是将优化后结果给了mInitFrame.mvpMapPoint，后面尺度固定后，再全部重新给一遍

            //尺度求解
            float s = ComputeSToMapPointDepth(4);
            float s_inv = 1/s;
            ROS_INFO_STREAM("s_inv: " << s_inv << "\ts: " << s);

            for(Frame* F : mFrame)
            {
                cv::Mat TT = F->GetPose();
                TT.rowRange(0,3).col(3) = TT.rowRange(0,3).col(3) * s_inv;
                F->SetPose(TT);
//                ROS_WARN_STREAM("TT: " << TT);

                //将尺度固定到位姿和地图点位置
                vector<pair<MapPoint*,int>> mMapPoint = F->GetmvpMapPoint();
                F->EarseAllMapPoint();
                for(int j = 0; j < mMapPoint.size(); j++)
                {
                    MapPoint* pp = mMapPoint[j].first;

                    cv::Mat pp3d = pp->GetWorldPos();
                    if(!pp->pflag)
                    {
                        pp3d *= s_inv;
                        pp->pflag = true;
                    }
                    pp->SetWorldPos(pp3d);

                    F->AddMapPoint(pp,mMapPoint[j].second);
//                    ROS_WARN_STREAM("********");
//                    ROS_WARN_STREAM("pp: " << pp->GetWorldPos());
//                    ROS_WARN_STREAM("idx: " << mMapPoint[j].second);
                }
            }
            //初始化完成
            FindKeyFrame = true;    //发送数据的标志位
            Init_send = true;       //证明发送的是初始化之后的数据的标志位
            mCurrentFrame.is_keyframe = true;
            mLastFrame = Frame(mCurrentFrame);

            Frame *Newkey = new Frame(mCurrentFrame);
            //这块的赋值，是深拷贝还是浅拷贝，有点懵  TODO
            //以防万一，我直接new一个新的，然后传递指针
            mapKeyFrame.insert(make_pair(mCurrentFrame.mid, Newkey));

//            NewKeyFrame.first = true;

        }
        else{
            InitFrame_First = true;
            return;
        }

        ROS_INFO_STREAM("Init Finish!");
        InitFinish = true;
    }
}

int Tracking::SearchForInit(vector<cv::DMatch> &good_matches) {

    ROS_DEBUG_STREAM("*****************************************************");
    ROS_DEBUG_STREAM("Descriptor Matcher!");
    int nmatches = 0;
    vector<cv::DMatch> matches_dist;

    //暴力匹配
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
    vector<cv::DMatch> matches;
    matcher->match(mInitFrame.mDescriptors,mCurrentFrame.mDescriptors,matches);
    ROS_DEBUG_STREAM("Descriptor Matcher!");
    //××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××
    //第一步筛选，根据描述子距离在30以内
    auto min_max = minmax_element(matches.begin(), matches.end(),[](const cv::DMatch &m1, const cv::DMatch &m2)
    { return m1.distance < m2.distance; });
    ROS_DEBUG_STREAM("Descriptor Matcher!");
    double min_dist = min_max.first->distance;
    double max_dist = min_max.second->distance;
    ROS_INFO_STREAM("min_dist: " << min_dist << "\tmax_dist: " << max_dist);
    ROS_DEBUG_STREAM("Descriptor Matcher!");
    //使用30作为阈值
    vector<cv::DMatch> matches_minmax;
    for(int i = 0; i < mInitFrame.mDescriptors.rows; i++){
        if(matches[i].distance <= max(2, 30)){
            matches_minmax.push_back(matches[i]);
        }
    }
    ROS_INFO_STREAM("1->matches_minmax.size: " << matches_minmax.size());

    //××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××
    //第二步筛选，使用旋转直方图剔除不合群的匹配
    //1.将360度建立成30个直方图bin
    vector<int> rotHist[HIST_LENGTH];
    //2.初步定义每个直方图空间位500
    for(int i1 = 0; i1 < HIST_LENGTH; i1++)
        rotHist[i1].reserve(500);

    const float factor = HIST_LENGTH/360.0f;
    //3.根据每个点的angle将其放置在不同的直方图bin中
    for(int idx = 0; idx < matches_minmax.size(); idx++){
        cv::KeyPoint pMP1 = mInitFrame.mKeys[matches_minmax[idx].queryIdx];
        cv::KeyPoint pMP2 = mCurrentFrame.mKeys[matches_minmax[idx].trainIdx];

        float rot = pMP1.angle - pMP2.angle;
        if(rot < 0)
            rot += 360;
        int bin = round(rot*factor);
        if(bin == HIST_LENGTH)
            bin = 0;
        rotHist[bin].push_back(idx);
    }
    //4.将直方图内数量最多的前三个bin中的点存起来，即得到的结果
    int ind1=-1;
    int ind2=-1;
    int ind3=-1;
    // 筛选出在旋转角度差落在在直方图区间内数量最多的前三个bin的索引
    ComputeThreeMaxima(rotHist,HIST_LENGTH,ind1,ind2,ind3);

    for(int i2 = 0; i2 < HIST_LENGTH; i2++){
        if(i2 == ind1 || i2 == ind2 || i2 == ind3){
            for(size_t j = 0; j < rotHist[i2].size(); j++){
                matches_dist.push_back(matches_minmax[rotHist[i2][j]]);
                nmatches++;
            }
        }
//        ROS_INFO_STREAM("rotHist[" << i2 << "].size: " << rotHist[i2].size());
    }
    ROS_INFO_STREAM("2->matches_dist.size: " << matches_dist.size());

    //第三步筛选，去除跨度较大的误匹配
    //调试过程中发现，到这里依然还是有一些明显的误匹配，初步使用两帧之间匹配点y值跨度不应该太大设置
    //这里的设置其实不太合理，因为图片的下半部分和上半部分在相机移动时跨度不相同（很大，近大远小）
    //后续看看效果再考虑是否更改   TODO
    for(int i3 = 0; i3 < matches_dist.size(); i3++){
        cv::Point2d feature_p1 = mInitFrame.mKeys[matches_dist[i3].queryIdx].pt;
        cv::Point2d feature_p2 = mCurrentFrame.mKeys[matches_dist[i3].trainIdx].pt;
        //使用的30作为跨度
//        if(abs(feature_p1.y - feature_p2.y) < 40)
        {
            good_matches.push_back(matches_dist[i3]);
        }
    }
    ROS_INFO_STREAM("3->good_matches.size: " << good_matches.size());

//    ROS_INFO_STREAM("p1.pt: [" << mInitFrame.mKeys[good_matches[118].queryIdx].pt.x << " , " << mInitFrame.mKeys[good_matches[118].queryIdx].pt.y << "]");
//    ROS_INFO_STREAM("p2.pt: [" << mCurrentFrame.mKeys[good_matches[118].trainIdx].pt.x << " , " << mCurrentFrame.mKeys[good_matches[118].trainIdx].pt.y << "]");
//    ROS_INFO_STREAM("matches_dist.size: " << matches_dist.size());
//    ROS_INFO_STREAM("good_matches.size: " << good_matches.size());
//    cv::Mat img_match;
//    cv::Mat img_goodmatch;
//    cv::drawMatches(mInitFrame.mFrameImg,mInitFrame.mKeys,mCurrentFrame.mFrameImg,mCurrentFrame.mKeys,match_1,img_match);
//    cv::drawMatches(mInitFrame.mFrameImg,mInitFrame.mKeys,mCurrentFrame.mFrameImg,mCurrentFrame.mKeys,good_matches,img_goodmatch);
//    cv::imshow("dist matches", img_match);
//    cv::imshow("good matches", img_goodmatch);
//    cv::waitKey(0);

    ROS_DEBUG_STREAM("*****************************************************");
    return good_matches.size();

}

void Tracking::ComputeThreeMaxima(vector<pair<int, pair<int, int>>>* histo, const int L, int &ind1, int &ind2, int &ind3)
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


void Tracking::ComputeThreeMaxima(vector<int>* histo, const int L, int &ind1, int &ind2, int &ind3)
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


bool Tracking::Initialization(vector<cv::DMatch> &good_matches, cv::Mat &R21, cv::Mat &t21,
                              vector<cv::Point3f> &point_w3d) {

    ROS_DEBUG_STREAM("*****************************************************");
    ROS_DEBUG_STREAM("Compute EssentialMat And R,t!");
    cv::Point2f pt;
    cv::Mat E21,mk;
    cv::Mat mask_3D,points_3D;
    int pCount = (int)good_matches.size();
    vector<cv::DMatch> match_opt;
    cv::Mat p1(pCount,2,CV_32F);
    cv::Mat p2(pCount,2,CV_32F);
    vector<cv::Point2f> p11,p22;
    yamlFile.mk.convertTo(mk,CV_32F);
    vector<unsigned char> RANSASStatus(good_matches.size());

    //使用findEssentialMat直接求解了E矩阵，其中mask就是外点
    //这个外点是使用了点到极线的距离判断，这个阈值3.84是根据卡方检验的值设置（好像是1自由度）
    for (int i=0; i<pCount; i++)
    {
        pt = mInitFrame.mKeys[good_matches[i].queryIdx].pt;
        p1.at<float>(i, 0) = pt.x;
        p1.at<float>(i, 1) = pt.y;

        pt = mCurrentFrame.mKeys[good_matches[i].trainIdx].pt;
        p2.at<float>(i, 0) = pt.x;
        p2.at<float>(i, 1) = pt.y;
//        ROS_INFO_STREAM("p1: " << p1.row(i) << "\t --> \tp2: " << p2.row(i) );
    }

    E21 = cv::findEssentialMat(p1,p2,mk,CV_RANSAC,0.999,3.84,RANSASStatus);
    ROS_INFO_STREAM("E21: " << E21.reshape(0,1));

    int InlinerCount = 0;
    for(int i1 = 0; i1 < pCount; i1++){
        if(RANSASStatus[i1]){
            InlinerCount++;
            match_opt.push_back(good_matches[i1]);
        }
    }

    ROS_INFO_STREAM("InlinerCount: " << InlinerCount);
    if(InlinerCount < 150){
        ROS_WARN_STREAM("InlinerCount: " << InlinerCount << " < 150");
        return false;
    }

//    cv::Mat img_in;
//    cv::Mat img_out;
//    cv::drawMatches(mInitFrame.mFrameImg,mInitFrame.mKeys,mCurrentFrame.mFrameImg,mCurrentFrame.mKeys,match_in,img_in);
//    cv::drawMatches(mInitFrame.mFrameImg,mInitFrame.mKeys,mCurrentFrame.mFrameImg,mCurrentFrame.mKeys,match_out,img_out);
//    cv::imshow("img_in", img_in);
//    cv::imshow("img_out",img_out);
//    cv::waitKey(0);

    for(int i2 = 0; i2 < match_opt.size(); i2++){
        p11.push_back(mInitFrame.mKeys[match_opt[i2].queryIdx].pt);
        p22.push_back(mCurrentFrame.mKeys[match_opt[i2].trainIdx].pt);
    }

    //分解E矩阵，得到了Rt，但是这里也进行了三角化，得到了地图点
    cv::recoverPose(E21,p11,p22,mk,R21,t21,50,mask_3D,points_3D);
    R21.convertTo(R21,CV_32F);
    t21.convertTo(t21,CV_32F);

    //opencv源码中这个mask只是判断了z轴正负和是否超过阈值distanceThresh，这个50是源码中的默认值（没有写在声明，在定义函数中）
    ROS_INFO_STREAM("R21: " << R21.reshape(0,1));
    ROS_INFO_STREAM("t21: " << t21.reshape(0,1));
    points_3D.convertTo(points_3D,CV_32F);

    points_3D.row(0) /= points_3D.row(3);
    points_3D.row(1) /= points_3D.row(3);
    points_3D.row(2) /= points_3D.row(3);
//    points_3D.row(3) /= points_3D.row(3);
    cv::Mat p3d_Mat = points_3D.rowRange(0,3);

    int point_num = 0;
    cv::Mat O1 = cv::Mat::zeros(3,1,CV_32F);
    cv::Mat O2 = -R21.t() * t21;
    cv::Mat pMP3d1 = cv::Mat::zeros(3,1,CV_32F);
    cv::Mat pMP3d2 = cv::Mat::zeros(3,1,CV_32F);
    ROS_INFO_STREAM("O2: " << O2.reshape(0,1));
    float pn_x = 0.0;
    //这里必须clear一下，都是教训。。找了好久。。
    match_3d2d.clear();

    //对每个地图点进行合法性检查
    for(int i3 = 0; i3 < p3d_Mat.cols; i3++){

        if(!(mask_3D.at<uchar>(i3,0)))
            continue;

        if(!isfinite(points_3D.at<float>(0,i3)) ||
           !isfinite(points_3D.at<float>(1,i3)) ||
           !isfinite(points_3D.at<float>(2,i3)))
            continue;

        pMP3d1 = p3d_Mat.col(i3);
        cv::Point2f p1_2d = p11[i3];
        cv::Point2f p2_2d = p22[i3];

        //视差角应该稍微大一点
        cv::Mat norm1 = pMP3d1 - O1;
        cv::Mat norm2 = pMP3d1 - O2;
        float dist1 = cv::norm(norm1);
        float dist2 = cv::norm(norm2);
        float cosPara = (norm1.dot(norm2))/(dist1*dist2);
        if(cosPara > 0.99999) //0.8度
            continue;

        //重投影误差，第一帧的误差
        float fea1x = (yamlFile.fx * pMP3d1.at<float>(0)) / pMP3d1.at<float>(2) + yamlFile.cx;
        float fea1y = (yamlFile.fy * pMP3d1.at<float>(1)) / pMP3d1.at<float>(2) + yamlFile.cy;
        float proError1 = (fea1x-p1_2d.x)*(fea1x-p1_2d.x) + (fea1y-p1_2d.y)*(fea1y-p1_2d.y);
        if(proError1 > 4.0)
            continue;

        //重投影误差，第二帧的误差
        pMP3d2 = R21 * pMP3d1 + t21;
        float fea2x = (yamlFile.fx * pMP3d2.at<float>(0)) / pMP3d2.at<float>(2) + yamlFile.cx;
        float fea2y = (yamlFile.fy * pMP3d2.at<float>(1)) / pMP3d2.at<float>(2) + yamlFile.cy;
        float proError2 = (fea2x-p2_2d.x)*(fea2x-p2_2d.x) + (fea2y-p2_2d.y)*(fea2y-p2_2d.y);
        if(proError2 > 4.0)
            continue;

        cv::Point3f p3d = cv::Point3f(pMP3d1.at<float>(0),pMP3d1.at<float>(1),pMP3d1.at<float>(2));
        point_w3d.push_back(p3d);

        pair<int, int> pp_idx = make_pair(match_opt[i3].queryIdx,match_opt[i3].trainIdx);
        match_3d2d.push_back(pp_idx);

//        ROS_INFO_STREAM("p3d: " << p3d << "\tp2d: " << " mInit:" << mInitFrame.mKeys[match_opt[i3].queryIdx].pt
//                                                    << " \tmCurr:" << mCurrentFrame.mKeys[match_opt[i3].trainIdx].pt
//                                                    << " \tIni-idx: " << match_opt[i3].queryIdx
//                                                    << " \tCur-idx: " << match_opt[i3].trainIdx);
        point_num++;
        pn_x += pMP3d1.at<float>(0);

    }
//    ROS_INFO_STREAM("pn_x: " << pn_x);
    ROS_INFO_STREAM("MapPoint_num: " << point_num);

    ROS_DEBUG_STREAM("*****************************************************");
    if(point_num < 100)
        return false;

//    for(auto match32 : match_3d2d){
//        ROS_INFO_STREAM("q: " << match32.first << "\tt: " << match32.second);
//    }

    return true;
}


float Tracking::ComputeSToMapPointDepth(int q) {

    vector<pair<MapPoint*,int>> InitmvpMP = mInitFrame.GetmvpMapPoint();
    cv::Mat Tcw = mCurrentFrame.GetPose();

    vector<float> zdepth;
    cv::Mat R_r2 = Tcw.row(2).colRange(0,3).t();
    float t_2 = Tcw.at<float>(2,3);
    for(int i = 0; i < InitmvpMP.size(); i++){

        MapPoint *pMP = InitmvpMP[i].first;
        float z = R_r2.dot(pMP->GetWorldPos()) + t_2;
        zdepth.push_back(z);
    }

    sort(zdepth.begin(),zdepth.end());

    float z_q = zdepth[floor(zdepth.size()/q)];

    return z_q;
}

//目前问题，可能会出现多个地图点匹配到同一个特征点的情况
int Tracking::SearchForTrack(float th_x, float th_y, bool ComeKeyFrame)
{
//    ROS_DEBUG_STREAM("*****************************************************");
    int nmatches = 0;
    vector<pair<MapPoint *, int>> mLastMapPoint;
    if(ComeKeyFrame)
        mLastMapPoint = mKeyFrame->GetmvpMapPoint();
    else
        mLastMapPoint = mLastFrame.GetmvpMapPoint();

    ROS_INFO_STREAM("feature_tracking => SearchForTrack:"
                    " x" << th_x << " y" << th_y <<
                    " ComeKeyFrame: " << ComeKeyFrame <<
                    "MapPoint.size: " << mLastMapPoint.size());

    //1.将360度建立成30个直方图bin
    vector<pair<int, pair<int ,int>>> rotHist[HIST_LENGTH];
    //2.初步定义每个直方图空间位500
    for(int i1 = 0; i1 < HIST_LENGTH; i1++)
        rotHist[i1].reserve(150);

    const float factor = HIST_LENGTH/360.0f;
    int HIGH_DIS = 40;

    cv::Mat mCR = mCurrentFrame.GetPose().rowRange(0,3).colRange(0,3);
    cv::Mat mCt = mCurrentFrame.GetPose().rowRange(0,3).col(3);

    map<int, pair<float, float>> mProCurrKeyPoint;
    //将每一个地图点投影到当前帧中
    for(int i = 0; i < mLastMapPoint.size(); i++)
    {
        MapPoint* mLmp = mLastMapPoint[i].first;

//        if(!ComeKeyFrame)
//            if(mLastFrame.KeyPointisMapPoint[mLastMapPoint[i].second] == true)
//                continue;

        //将上一帧的地图点投影到当前帧
        cv::Mat mLmpPos = mLmp->GetWorldPos();
        cv::Mat mCmpPos = mCR * mLmpPos + mCt;
        float mCmp_u = yamlFile.fx * (mCmpPos.at<float>(0) / mCmpPos.at<float>(2)) + yamlFile.cx;
        float mCmp_v = yamlFile.fy * (mCmpPos.at<float>(1) / mCmpPos.at<float>(2)) + yamlFile.cy;

        int bestDis = 256;
        int bestDis2 = 256;
        int bestDis_idx = -1;
        for(int j = 0; j < mCurrentFrame.mDescriptors.rows; j++)
        {
            //指定了一个范围，再这个长方形的范围内搜索
            //这个搜索有个想法，可以使用四茶树均匀化时候的树结构，然后进行树结构向上搜索
            if(mCurrentFrame.KeyPointisMapPoint[j])
                continue;

            if(abs(mCurrentFrame.mKeys[j].pt.x - mCmp_u) < th_x &&
                abs(mCurrentFrame.mKeys[j].pt.y - mCmp_v) < th_y)
            {
                cv::Mat mLmp_descriptor;
                if(ComeKeyFrame)
                    mLmp_descriptor = mKeyFrame->mDescriptors.row(mLastMapPoint[i].second);
                else
                    mLmp_descriptor = mLastFrame.mDescriptors.row(mLastMapPoint[i].second);
//                ROS_INFO_STREAM("mLmp_des: " << mLmp_descriptor);
                cv::Mat mCmp_descriptor = mCurrentFrame.mDescriptors.row(j);
//                ROS_INFO_STREAM("mCmp_des: " << mCmp_descriptor);
                //计算描述子距离，hanming距离就是两个32字节中不同位的个数
                int dis_descriptor = DescriptorDistance(mLmp_descriptor,mCmp_descriptor);
                //选取最好的描述子距离，这里看了下效果还是可以了，因此没有使用最优/次优
                if(dis_descriptor < bestDis)
                {
                    bestDis2 = bestDis;
                    bestDis = dis_descriptor;
                    bestDis_idx = j;
                }
                else if(dis_descriptor < bestDis2)
                {
                    bestDis2 = dis_descriptor;
                }
//                ROS_INFO_STREAM("dis_descriptor: " << dis_descriptor << "\tj: " << j);
            }
        }

        if(bestDis > (float)bestDis2*0.8)
            continue;

        //如果这个距离小于一定阈值
        if(bestDis < HIGH_DIS)
        {
            cv::KeyPoint mLpMP;
            if(ComeKeyFrame)
                mLpMP = mKeyFrame->mKeys[mLastMapPoint[i].second];
            else
                mLpMP = mLastFrame.mKeys[mLastMapPoint[i].second];

            cv::KeyPoint mCpMP = mCurrentFrame.mKeys[bestDis_idx];

            float rot = mLpMP.angle - mCpMP.angle;
            if(rot < 0)
                rot += 360;
            int bin = round(rot*factor);
            if(bin == HIST_LENGTH)
                bin = 0;
            auto LC_idx = make_pair(i, make_pair(bestDis_idx,bestDis));
            rotHist[bin].push_back(LC_idx);
//            mCurrentFrame.KeyPointisMapPoint[bestDis_idx] = true;

            mProCurrKeyPoint.insert(make_pair(bestDis_idx, make_pair(mCmp_u, mCmp_v)));
        }
//        ROS_WARN_STREAM("mLmpId: " << mLmp->mid <<
//                        "\tbestDis: " << bestDis <<
//                        "\tbestDis_idx: " << bestDis_idx);
    }

    //测试使用
    vector<pair<float, float>> mProKeyPoint;

    map<int,pair<int,int>> MatchKeyPoint;   //当前帧索引，mLastMapPoint索引，描述子距离
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
                MapPoint* p3d = mLastMapPoint[rotHist[i2][j].first].first;
                int mCidx = rotHist[i2][j].second.first;
                int mCdistance = rotHist[i2][j].second.second;

                //下面的一通操作都是为了防止出现多个地图点匹配上同一个特征点的情况
                if(MatchKeyPoint.empty())
                {
                    MatchKeyPoint.insert(make_pair(mCidx, make_pair(rotHist[i2][j].first,mCdistance)));
                    continue;
                }

                auto MatchPos = MatchKeyPoint.find(mCidx);
                if(MatchPos != MatchKeyPoint.end())
                {
                    if(mCdistance <= MatchPos->second.second)
                        MatchKeyPoint[mCidx] = make_pair(rotHist[i2][j].first,mCdistance);
                }
                else
                    MatchKeyPoint.insert(make_pair(mCidx, make_pair(rotHist[i2][j].first,mCdistance)));

//                mCurrentFrame.AddMapPoint(p3d,mCidx);
//                mCurrentFrame.KeyPointisMapPoint[mCidx] = true;
//                ROS_INFO_STREAM("p3d: " << p3d->GetWorldPos().reshape(0,1) <<
//                                "\tmCidx: " << mCidx);

                nmatches++;
            }
        }
//        ROS_INFO_STREAM("rotHist[" << i2 << "].size: " << rotHist[i2].size());
    }

    for(auto it = MatchKeyPoint.begin(); it != MatchKeyPoint.end(); it++)
    {
        MapPoint* pMP = mLastMapPoint[it->second.first].first;
        mCurrentFrame.AddMapPoint(pMP, it->first);
        mCurrentFrame.KeyPointisMapPoint[it->first] = true;
//        if(!ComeKeyFrame)
//            mLastFrame.KeyPointisMapPoint[it->second.first] = true;

        auto iidx = make_pair(mLastMapPoint[it->second.first].second,it->first);
        MapPoint_iidx.push_back(iidx);
        auto mPro = mProCurrKeyPoint.find(it->first);
        mProKeyPoint.push_back(make_pair(mPro->second.first, mPro->second.second));
    }


    ROS_INFO_STREAM("feature_tracking => nmatches: " << nmatches);

    //下面是调试，可查看匹配的图像
//    if(mCurrentFrame.mid >= 120)
//    {
//        float i = 0.0;
//        vector<cv::DMatch> matches_test;
//        for(auto midx : MapPoint_iidx)
//        {
//            cv::DMatch match_idx = cv::DMatch(midx.first,midx.second,i++);
//            matches_test.push_back(match_idx);
//        }
//
//        for(int k = 0; k < mProKeyPoint.size(); k++)
//        {
//            cv::Point2f p2f = cv::Point2f(mProKeyPoint[k].first, mProKeyPoint[k].second);
////        ROS_INFO_STREAM("p2f: " << p2f);
//            cv::KeyPoint p3k = cv::KeyPoint(p2f,0,0,0,0);
//            circle(mCurrentFrame.mFrameImg, cv::Point(p3k.pt.x,p3k.pt.y), 2, cv::Scalar(0, 255, 0), -1);
//        }
//
//        cv::Mat img_in;
//        if(ComeKeyFrame)
//            cv::drawMatches(mKeyFrame->mFrameImg,mKeyFrame->mKeys,mCurrentFrame.mFrameImg,mCurrentFrame.mKeys,matches_test,img_in);
//        else
//            cv::drawMatches(mLastFrame.mFrameImg,mLastFrame.mKeys,mCurrentFrame.mFrameImg,mCurrentFrame.mKeys,matches_test,img_in);
//        cv::imshow("img_in", img_in);
//        cv::waitKey(0);
//    }


//    ROS_DEBUG_STREAM("*****************************************************");
    return nmatches;
}


void Tracking::Track() {

    float th_x = mCurrentFrame.mFrameImg.cols/10;
    float th_y = mCurrentFrame.mFrameImg.rows/10;
//    ROS_INFO_STREAM("mid: " << mCurrentFrame.mid);

    //设置当前帧位姿的初始值
    //如果是初始化之后的第一帧，那么就将初始值设置为与上一帧一样
    //如果是跟踪过程，就根据恒速设置初始值
    if(mCurrentFrame.mid - mInitFrame.mid == 2)
    {
        mCurrentFrame.SetPose(mLastFrame.GetPose());
//        th_x = 60.0;
//        th_y = 40.0;
    }
    else
        mCurrentFrame.SetPose(mLastFrame.GetPose()*mVelocity);

    //进行特征匹配3D-2D的匹配
    //上一阵地图点投影到当前帧，然后进行搜索，再根据描述子进行匹配
    mCurrentFrame.EarseAllMapPoint();
    MapPoint_iidx.clear();
    int NumMatch = SearchForTrack(th_x, th_y,false);
    if(NumMatch < 30)
    {
//        mCurrentFrame.KeyPointisMapPoint.resize(mCurrentFrame.mKeys.size(), false);
//        mCurrentFrame.EarseAllMapPoint();
        NumMatch = SearchForTrack(th_x*2,th_y*1.5, NewKeyFrame.first);
    }

    //匹配之后，就不需要了


    //g2o优化，只优化当前帧位姿
    vector<pair<MapPoint*, int>> mnMapPoints;
    mnMapPoints = mCurrentFrame.GetmvpMapPoint();
    TrackingBA(&mCurrentFrame,mnMapPoints);

    mCurrentFrame.EarseAllMapPoint();
    for(int i = 0; i < mnMapPoints.size(); i++)
    {
        MapPoint* pMP = mnMapPoints[i].first;
        int idx = mnMapPoints[i].second;
        if(!pMP->is_outlier)
            mCurrentFrame.AddMapPoint(pMP,idx);
        else
            mCurrentFrame.KeyPointisMapPoint[idx] = false;
    }
    mCurrTrackMapPointSize = mCurrentFrame.GetmvpMapPoint().size();

    ROS_INFO_STREAM("mnMapPoints: " << mCurrTrackMapPointSize);
    ROS_INFO_STREAM("feature_tracking => mCurrFrame MapPoint size: " << mCurrentFrame.mvpMapPoint.size());
    ROS_INFO_STREAM("feature_tracking => mCurrFrame pose: " << mCurrentFrame.GetPose().reshape(0,1));

//    if(mCurrentFrame.mid-mInitFrame.mid < 10)
//    {
//        float i = 0.0;
//        vector<cv::DMatch> matches_test;
//        for(auto midx : MapPoint_iidx)
//        {
//            cv::DMatch match_idx = cv::DMatch(midx.first,midx.second,i++);
//            matches_test.push_back(match_idx);
//        }
//        auto aa = mCurrentFrame.GetmvpMapPoint();
//        for(auto a : aa)
//        {
//            cv::KeyPoint kp = mCurrentFrame.mKeys[a.second];
//            circle(mCurrentFrame.mFrameImg, cv::Point(kp.pt.x,kp.pt.y), 6, cv::Scalar(0, 255, 0), 6);
//        }
//
//
//        cv::Mat img_in;
//        cv::drawMatches(mLastFrame.mFrameImg,mLastFrame.mKeys,mCurrentFrame.mFrameImg,mCurrentFrame.mKeys,matches_test,img_in);
//        cv::imshow("img_in", img_in);
//        cv::waitKey(0);
//    }
//    auto mMapPoint = mCurrentFrame.GetmvpMapPoint();
//    for(int j = 0; j < mMapPoint.size(); j++)
//    {
//        MapPoint* MP = mMapPoint[j].first;
//        ROS_WARN_STREAM("MP.id: " << MP->mid << "\tpos: " << MP->GetWorldPos().reshape(0,1));
//    }
}


void Tracking::LocalMapTrack()
{
    float th_x = mCurrentFrame.mFrameImg.cols/10;
    float th_y = mCurrentFrame.mFrameImg.rows/10;
    ROS_WARN_STREAM("LocalMapTrack");

    //设置当前帧位姿的初始值
    //如果是初始化之后的第一帧，那么就将初始值设置为与上一帧一样
    //如果是跟踪过程，就根据恒速设置初始值
    if(mCurrentFrame.mid - mInitFrame.mid == 2)
        return;

    vector<Frame*> mCoviewingFrame;

    auto it = mapKeyFrame.find(mCurrentFrame.mReferKeyFrameId);
    if(it == mapKeyFrame.end())
    {
        ROS_ERROR_STREAM("Can not Find mReferKeyFrame");
        return;
    }
    Frame* ReferKeyFrame = it->second;
    if(ReferKeyFrame->mid != mCurrentFrame.mReferKeyFrameId)
    {
        ROS_ERROR_STREAM("Find mReferKeyFrame is not right");
        return;
    }

    if(!mLastFrame.is_keyframe)
        mCoviewingFrame.push_back(ReferKeyFrame);

//    ROS_WARN_STREAM("mReferKeyFrameId: " << ReferKeyFrame->GetPose());
//    ROS_WARN_STREAM("mCoviewFrameId.size: " << ReferKeyFrame->mCoviewFrameId.size());

    for(int i = 0; i < ReferKeyFrame->mCoviewFrameId.size(); i++)
    {
        auto mReferCoview = mapKeyFrame.find(ReferKeyFrame->mCoviewFrameId[i]);
        if(mReferCoview == mapKeyFrame.end())
        {
            ROS_ERROR_STREAM("Can not Find mReferCoview");
            continue;
        }
        mCoviewingFrame.push_back(mReferCoview->second);
    }


    for(Frame* mReferCoviewFrame : mCoviewingFrame)
    {
        mKeyFrame = mReferCoviewFrame;

//        ROS_WARN_STREAM("mKeyFrame.id: " << mKeyFrame->mid);
        //进行特征匹配3D-2D的匹配
        //上一阵地图点投影到当前帧，然后进行搜索，再根据描述子进行匹配
        int NumMatch = SearchForTrack(th_x, th_y, true);
        ROS_WARN_STREAM("NumMatch: " << NumMatch);
    }

    //使用map容器滤一下，去掉某个地图点匹配到多个特征点的情况
    //其实mvpMapPoint就应该使用map，但是在后期发现了问题，修改工程量比较大
    //先进来的应该是比较靠近的帧，相对更准一些
    //所以后进来的，就不修改之前的值了
    map<unsigned long, pair<MapPoint*, int>> FilterMapPoint;
    auto mMapPoint = mCurrentFrame.GetmvpMapPoint();
    for(int j = 0; j < mMapPoint.size(); j++)
    {
        MapPoint* MP = mMapPoint[j].first;
        FilterMapPoint.insert(make_pair(MP->mid, make_pair(MP, mMapPoint[j].second)));
//        ROS_WARN_STREAM("LocalMapTrack-MP.id: " << MP->mid << "\tpos: " << MP->GetWorldPos().reshape(0,1) << "\tidx: " << mMapPoint[j].second);
    }
    mCurrentFrame.EarseAllMapPoint();
    for(auto it = FilterMapPoint.begin(); it != FilterMapPoint.end(); it++)
        mCurrentFrame.AddMapPoint(it->second.first, it->second.second);


    //g2o优化，只优化当前帧位姿
    vector<pair<MapPoint*, int>> mnMapPoints;
    mnMapPoints = mCurrentFrame.GetmvpMapPoint();
    ROS_WARN_STREAM("before-TrackingBA: " << mnMapPoints.size());
    TrackingBA(&mCurrentFrame,mnMapPoints);

    mCurrentFrame.EarseAllMapPoint();
    for(int i = 0; i < mnMapPoints.size(); i++)
    {
        MapPoint* pMP = mnMapPoints[i].first;
        int idx = mnMapPoints[i].second;
        if(!pMP->is_outlier)
            mCurrentFrame.AddMapPoint(pMP,idx);
        else
            mCurrentFrame.KeyPointisMapPoint[idx] = false;
    }

    ROS_INFO_STREAM("feature_tracking => mCurrFrame MapPoint size: " << mCurrentFrame.mvpMapPoint.size());
    ROS_INFO_STREAM("feature_tracking => mCurrFrame pose: " << mCurrentFrame.GetPose().reshape(0,1));
}



// http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
int Tracking::DescriptorDistance(const cv::Mat &a, const cv::Mat &b)
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

bool Tracking::GetNewFrame(cv::Mat &img, double &imgtime)
{
    m_frame.lock();
    if(mListNewFrame.empty())
    {
        m_frame.unlock();
        return false;
    }

    auto img_msg = mListNewFrame.front();
    img = img_msg.first;
    imgtime = img_msg.second;
    mListNewFrame.pop_front();
    m_frame.unlock();

    return true;

}

