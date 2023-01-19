
#include "Optimizer.h"

void LocalMapBA(vector<KeyFrame*> &LocalKeyFrames, KeyFrame* CurrKeyFrame, vector<KeyFrame*> &NewPartKeyFrame, vector<MapPoint*> &NewMapPoints, cv::Mat &K)
{
    vector<KeyFrame*> OptKeyFrames;
    vector<KeyFrame*> UnOptKeyFrames;
    vector<MapPoint*> OptMapPoints;

    OptKeyFrames.push_back(CurrKeyFrame);
//    for(auto NewPartKF : NewPartKeyFrame)
//    {
//        if(NewPartKF->ParkInOptKeyFrameId != CurrKeyFrame->mid)
//        {
//            OptKeyFrames.push_back(NewPartKF);
//            NewPartKF->ParkInOptKeyFrameId = CurrKeyFrame->mid;
//        }
//    }

    for(int i = 0; i < LocalKeyFrames.size(); i++)
    {
        int max_weight = CurrKeyFrame->mCoviewing.front().second;
        if(CurrKeyFrame->mCoviewing[i].second > 0.5*max_weight)
        {
            OptKeyFrames.push_back(LocalKeyFrames[i]);
            ROS_WARN_STREAM("Opt-frameid: " << LocalKeyFrames[i]->frameId << "\tOpt-mid: " << LocalKeyFrames[i]->mid);
        }
        else
        {
            UnOptKeyFrames.push_back(LocalKeyFrames[i]);
            ROS_WARN_STREAM("UnOpt-frameid: " << LocalKeyFrames[i]->frameId << "\tUnOpt-mid: " << LocalKeyFrames[i]->mid);
        }

//        if(LocalKeyFrames[i]->ParkInOptKeyFrameId != CurrKeyFrame->mid)
//        {
//            UnOptKeyFrames.push_back(LocalKeyFrames[i]);
////            LocalKeyFrames[i]->ParkInOptKeyFrameId = CurrKeyFrame->mid;
//        }
    }

    for(int k = 0; k < NewMapPoints.size(); k++)
    {
        if(NewMapPoints[k]->mPartOptKeyFrameId != CurrKeyFrame->mid)
        {
            OptMapPoints.push_back(NewMapPoints[k]);
            NewMapPoints[k]->mPartOptKeyFrameId = CurrKeyFrame->mid;
        }
    }

    LocalKeyFrames.push_back(CurrKeyFrame);
    for(int j = 0; j < LocalKeyFrames.size(); j++)
    {
        map<unsigned long, pair<MapPoint*, int>> OptKFMP;
        OptKFMP = LocalKeyFrames[j]->GetmvpMapPoint();

        for(auto it0 = OptKFMP.begin(); it0 != OptKFMP.end(); it0++)
        {
            //如果出现这种情况，就是出现了问题，应该不会出现
            if(it0->first != it0->second.first->mid)
                continue;

            MapPoint* pKFMP = it0->second.first;
//            ROS_DEBUG_STREAM("pid: " << pKFMP->mid << "\tp: " << pKFMP->GetWorldPos());

            if(pKFMP->mPartOptKeyFrameId != CurrKeyFrame->mid)
            {
                OptMapPoints.push_back(pKFMP);
                pKFMP->mPartOptKeyFrameId = CurrKeyFrame->mid;
            }
        }
    }

    LocalMapG2O(OptKeyFrames,UnOptKeyFrames,OptMapPoints, K);
}

void LocalMapG2O(vector<KeyFrame*> &mOptKeyFrame, vector<KeyFrame*> &mFixKeyFrame, vector<MapPoint*> &mOptMapPoints, cv::Mat &K)
{

    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);
    std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver;
    linearSolver = g2o::make_unique<g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>>();

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
            g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver))
    );
    optimizer.setAlgorithm(solver);

    double focal_length = K.at<float>(0,0);
    Eigen::Vector2d principal_point(K.at<float>(0,2), K.at<float>(1,2));
//    ROS_WARN_STREAM("LocalMapping::mk.at<float>(0,0): " << K.at<float>(0,0));

    vector<g2o::SE3Quat, Eigen::aligned_allocator<g2o::SE3Quat>> true_poses;
    g2o::CameraParameters * cam_params = new g2o::CameraParameters (focal_length, principal_point, 0.);
    cam_params->setId(0);

    if (!optimizer.addParameter(cam_params)) {
        assert(false);
    }

//    ROS_INFO_STREAM("Vertex");
//    ROS_INFO_STREAM("p3d.size(): " << p3d.size());
    //定义顶点和边
    unsigned int maxid = 0;
    set<KeyFrame*> IsAdd;

    for(int i = 0; i < mOptKeyFrame.size(); i++)
    {
        g2o::VertexSE3Expmap *v1 = new g2o::VertexSE3Expmap();
        cv::Mat Tcw = mOptKeyFrame[i]->GetPose();
        v1->setEstimate(Mat2SE3Quat(Tcw));
        v1->setId(mOptKeyFrame[i]->mid);
        if(mOptKeyFrame[i]->mid == 0)
            v1->setFixed(true);

        if(maxid < mOptKeyFrame[i]->mid)
            maxid = mOptKeyFrame[i]->mid;

        optimizer.addVertex(v1);
        IsAdd.insert(mOptKeyFrame[i]);
//        ROS_WARN_STREAM("KF-id: " << mOptKeyFrame[i]->mid);
    }

    for(int j = 0; j < mFixKeyFrame.size(); j++)
    {
        g2o::VertexSE3Expmap *v2 = new g2o::VertexSE3Expmap();
        cv::Mat Tcw = mFixKeyFrame[j]->GetPose();
        v2->setEstimate(Mat2SE3Quat(Tcw));
        v2->setId(mFixKeyFrame[j]->mid);
        v2->setFixed(true);

        if(maxid < mFixKeyFrame[j]->mid)
            maxid = mFixKeyFrame[j]->mid;

        optimizer.addVertex(v2);
        IsAdd.insert(mFixKeyFrame[j]);
    }

    vector<g2o::EdgeProjectXYZ2UV*> EdgeChi2;
    vector<MapPoint*> MapPointChi2;

    for(int k = 0; k < mOptMapPoints.size(); k++)
    {
        MapPoint* pMP = mOptMapPoints[k];
        cv::Mat p3d_pose = pMP->GetWorldPos();

        Eigen::Vector3d p3d_opt(p3d_pose.at<float>(0), p3d_pose.at<float>(1), p3d_pose.at<float>(2));

        g2o::VertexSBAPointXYZ *point = new g2o::VertexSBAPointXYZ();
        point->setEstimate(p3d_opt);
        point->setId(pMP->mid + maxid + 1);
        point->setMarginalized(true);

        optimizer.addVertex(point);
//        ROS_DEBUG_STREAM("p3d_pose: " << p3d_pose);
//        ROS_WARN_STREAM("MP-id: " << pMP->mid + maxid + 1);
//        ROS_DEBUG_STREAM("p3d_opt: " << p3d_opt);
        map<KeyFrame*, int> CoviewKF;
        CoviewKF = pMP->GetObservation();
        for(auto k1 = CoviewKF.begin(); k1 != CoviewKF.end(); k1++)
        {
            KeyFrame* pKF = k1->first;
            int idx = k1->second;

            //这里用count也行
            set<KeyFrame*>::iterator pos = IsAdd.find(pKF);
            if(pos != IsAdd.end())
            {
                cv::KeyPoint p2d_pose = pKF->mKeys[idx];
                Eigen::Matrix<double, 2, 1> p2d_opt(p2d_pose.pt.x, p2d_pose.pt.y);

                g2o::EdgeProjectXYZ2UV *edge = new g2o::EdgeProjectXYZ2UV();

                edge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pMP->mid+maxid+1)));
                edge->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKF->mid)));
                edge->setMeasurement(p2d_opt);
//                ROS_DEBUG_STREAM("pid: " << pMP->mid + maxid + 1 << "\tkid: " << pKF->mid);
//                ROS_DEBUG_STREAM("p2d_opt: " << p2d_opt);

                const double sigma2 = 1 / pow(pow(1.2, 2), p2d_pose.octave);
                edge->setInformation(Eigen::Matrix2d::Identity() * sigma2);
                edge->setParameterId(0,0);

                g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                edge->setRobustKernel(rk);
                rk->setDelta(sqrt(5.99));

                optimizer.addEdge(edge);
                EdgeChi2.push_back(edge);
                MapPointChi2.push_back(pMP);
            }

        }
    }

    ROS_INFO_STREAM("local_map => optimizer: LocalMap");
//    ROS_WARN_STREAM("mOptKeyFrame: " << mOptKeyFrame.size());
//    ROS_WARN_STREAM("mFixKeyFrame: " << mFixKeyFrame.size());
//    ROS_WARN_STREAM("mOptMapPoints: " << mOptMapPoints.size());
//    ROS_INFO_STREAM("Edge");
    //设置优化参数，开始优化
    for(int i0 = 0; i0 < 2; i0++)
    {
        //设置优化参数，开始优化
        optimizer.initializeOptimization(0);
        optimizer.optimize(10+i0*5);

        for(int j0 = 0; j0 < EdgeChi2.size(); j0++)
        {
            g2o::EdgeProjectXYZ2UV* e = EdgeChi2[j0];
            MapPoint* pMP = MapPointChi2[j0];

            if(e->chi2() > 5.991)
            {
                e->setLevel(1);
                pMP->is_outlier = true;
            }
//            e->setRobustKernel(0);
        }
    }


    for(int i1 = 0; i1 < mOptKeyFrame.size(); i1++)
    {
        if(mOptKeyFrame[i1]->mid==0)
            continue;

        cv::Mat matF;
        g2o::VertexSE3Expmap* vF = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(mOptKeyFrame[i1]->mid));
        g2o::SE3Quat se3F = vF->estimate();
        Eigen::Matrix<double, 4, 4> eigenF = se3F.to_homogeneous_matrix();
        cv::eigen2cv(eigenF,matF);
        matF.convertTo(matF,CV_32F);
        mOptKeyFrame[i1]->SetPose(matF);
//        ROS_INFO_STREAM("matF: " << matF.reshape(0,1));
    }

    for(int i2 = 0; i2 < mOptMapPoints.size(); i2++)
    {
        MapPoint *pMP = mOptMapPoints[i2];
        if(pMP->is_outlier)
            continue;

        cv::Mat matP;
        g2o::VertexSBAPointXYZ* vP = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mid+maxid+1));
        Eigen::Matrix<double, 3, 1> eigenP = vP->estimate();
        cv::eigen2cv(eigenP,matP);
        matP.convertTo(matP,CV_32F);
        pMP->SetWorldPos(matP);
//        ROS_INFO_STREAM("matP: " << matP.reshape(0,1));
//        ROS_INFO_STREAM("idx: " << p3d[i4].second);
    }

//    ROS_DEBUG_STREAM("*****************************************************");

}

g2o::SE3Quat Mat2SE3Quat(cv::Mat &m2sTcw)
{
    Eigen::Matrix<double,3,3> R;
    R << m2sTcw.at<float>(0,0), m2sTcw.at<float>(0,1), m2sTcw.at<float>(0,2),
            m2sTcw.at<float>(1,0), m2sTcw.at<float>(1,1), m2sTcw.at<float>(1,2),
            m2sTcw.at<float>(2,0), m2sTcw.at<float>(2,1), m2sTcw.at<float>(2,2);
    Eigen::Matrix<double,3,1> t;
    t << m2sTcw.at<float>(0,3), m2sTcw.at<float>(1,3), m2sTcw.at<float>(2,3);
    return g2o::SE3Quat(R,t);
}