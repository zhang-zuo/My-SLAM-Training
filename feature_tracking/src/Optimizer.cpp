//
// Created by zhangzuo on 22-8-16.
//
#include <iostream>
#include "Optimizer.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/solvers/structure_only/structure_only_solver.h"
#include <opencv2/core/eigen.hpp>

#include<Eigen/StdVector>

extern yamlfile yamlFile;

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


void GlobalBAInit(vector<Frame *> FrameNum, vector<pair<MapPoint*, int>> &p3d)
{
//    ROS_DEBUG_STREAM("*****************************************************");
    ROS_INFO_STREAM("feature_tracking => GlobalBAInit to KeyFrame and MapPoints!");
    int maxid = FrameNum[1]->mid;
    float pp_xx = 0.0;

//    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,3>> Block;
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);
    std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver;
    linearSolver = g2o::make_unique<g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>>();

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
            g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver))
    );
    optimizer.setAlgorithm(solver);

    double focal_length = yamlFile.fx;
    Eigen::Vector2d principal_point(yamlFile.cx, yamlFile.cy);

    vector<g2o::SE3Quat, Eigen::aligned_allocator<g2o::SE3Quat>> true_poses;
    g2o::CameraParameters * cam_params = new g2o::CameraParameters (focal_length, principal_point, 0.);
    cam_params->setId(0);

    if (!optimizer.addParameter(cam_params)) {
        assert(false);
    }


//    ROS_INFO_STREAM("Vertex");
//    ROS_INFO_STREAM("p3d.size(): " << p3d.size());
    //定义顶点和边

    for(int i0 = 0; i0 < FrameNum.size(); i0++)
    {
        g2o::VertexSE3Expmap *v = new g2o::VertexSE3Expmap();
        cv::Mat Tcw = FrameNum[i0]->GetPose();
        v->setEstimate(Mat2SE3Quat(Tcw));
        v->setId(FrameNum[i0]->mid);
        if(i0 == 0)
            v->setFixed(true);
        optimizer.addVertex(v);
//        ROS_INFO_STREAM("T: " << Mat2SE3Quat(Tcw).to_homogeneous_matrix());
    }

    for(int i = 0; i < p3d.size(); i++)
    {
        MapPoint *pMP = p3d[i].first;

        if(pMP == NULL)
            continue;

        cv::Mat p3d_pose = pMP->GetWorldPos();
        Eigen::Vector3d p3d_opt(p3d_pose.at<float>(0), p3d_pose.at<float>(1), p3d_pose.at<float>(2));
//        Eigen::Vector3d p3d_opt(10, 10, 10);

        g2o::VertexSBAPointXYZ *point = new g2o::VertexSBAPointXYZ();

        point->setEstimate(p3d_opt);
        point->setId(pMP->mid + maxid + 1);
        point->setMarginalized(true);

        optimizer.addVertex(point);
    }

    for(int j = 0; j < FrameNum.size(); j++)
    {
        for(int j1 = 0; j1 < p3d.size(); j1++)
        {
            MapPoint *pMP = p3d[j1].first;

            if(pMP == NULL)
                continue;

            int p_idx = FrameNum[j]->mvpMapPoint[j1].second;
            cv::KeyPoint p2d_pose = FrameNum[j]->mKeys[p_idx];
            Eigen::Matrix<double, 2, 1> p2d_opt(p2d_pose.pt.x, p2d_pose.pt.y);

            g2o::EdgeProjectXYZ2UV *edge = new g2o::EdgeProjectXYZ2UV();

            edge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pMP->mid+maxid+1)));
            edge->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(FrameNum[j]->mid)));
            edge->setMeasurement(p2d_opt);

            const double sigma2 = 1 / pow(pow(yamlFile.scaleFactor, 2), p2d_pose.octave);
            edge->setInformation(Eigen::Matrix2d::Identity() * sigma2);
            edge->setParameterId(0,0);

            //鲁帮核函数
            g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
            edge->setRobustKernel(rk);
            rk->setDelta(sqrt(5.99));

            cv::Mat p3d_pose1 = pMP->GetWorldPos();
            Eigen::Vector3d p3d_opt1(p3d_pose1.at<float>(0), p3d_pose1.at<float>(1), p3d_pose1.at<float>(2));
//            ROS_WARN_STREAM("**************************************************");
//            ROS_WARN_STREAM("j: " << j << "\tpid: " << pMP->mid+maxid+1 << "\teid: " << FrameNum[j]->mid);
//            ROS_WARN_STREAM("octave: " << p2d_pose.octave << "\tsigma: " << sigma2);
//            ROS_WARN_STREAM("p3d: " << p3d_opt1);
//            ROS_WARN_STREAM("p2d: " << p2d_opt);
//            ROS_WARN_STREAM("chi2: " << edge->chi2());
//            ROS_WARN_STREAM("**************************************************");

            optimizer.addEdge(edge);
        }
    }

    ROS_INFO_STREAM("feature_tracking => optimizer: Init");
//    ROS_INFO_STREAM("Edge");
    //设置优化参数，开始优化
    optimizer.initializeOptimization();
    optimizer.optimize(20);


    for(int i3 = 0; i3 < FrameNum.size(); i3++)
    {
        if(!FrameNum[i3]->is_keyframe)
            continue;

        cv::Mat matF;
        g2o::VertexSE3Expmap* vF = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(FrameNum[i3]->mid));
        g2o::SE3Quat se3F = vF->estimate();
        Eigen::Matrix<double, 4, 4> eigenF = se3F.to_homogeneous_matrix();
        cv::eigen2cv(eigenF,matF);
        matF.convertTo(matF,CV_32F);
        FrameNum[i3]->SetPose(matF);
//        ROS_INFO_STREAM("matF: " << matF.reshape(0,1));
    }

    for(int i4 = 0; i4 < p3d.size(); i4++)
    {
        MapPoint *pMP = p3d[i4].first;

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


void TrackingBA(Frame *CurrFrame, vector<pair<MapPoint*, int>> &CurrMapPoint)
{
//    ROS_DEBUG_STREAM("*****************************************************");
    ROS_INFO_STREAM("feature_tracking => GlobalBAInit to KeyFrame and MapPoints!");

    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);
    std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver;
    linearSolver = g2o::make_unique<g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>>();

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
            g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver))
    );
    optimizer.setAlgorithm(solver);

    //定义顶点和边
    g2o::VertexSE3Expmap *v = new g2o::VertexSE3Expmap();
    cv::Mat Tcw = CurrFrame->GetPose();
    v->setEstimate(Mat2SE3Quat(Tcw));
    v->setId(0);
    optimizer.addVertex(v);
//    ROS_INFO_STREAM("T: " << Mat2SE3Quat(Tcw).to_homogeneous_matrix());

    vector<g2o::EdgeSE3ProjectXYZOnlyPose*> VectorEdge;
    VectorEdge.reserve(CurrMapPoint.size());

    for(int i = 0; i < CurrMapPoint.size(); i++)
    {
        MapPoint *pMP = CurrMapPoint[i].first;
//        if(pMP == NULL)
//            continue;

        cv::Mat p3d_pose = pMP->GetWorldPos();
        cv::KeyPoint p2d = CurrFrame->mKeys[CurrMapPoint[i].second];
        Eigen::Matrix<double, 2, 1> p2d_opt(p2d.pt.x, p2d.pt.y);

        g2o::EdgeSE3ProjectXYZOnlyPose *edge = new g2o::EdgeSE3ProjectXYZOnlyPose();
        edge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
        edge->setMeasurement(p2d_opt);

        const double sigma2 = 1 / pow(pow(yamlFile.scaleFactor, 2), p2d.octave);
        edge->setInformation(Eigen::Matrix2d::Identity() * sigma2);

        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
        edge->setRobustKernel(rk);
        rk->setDelta(sqrt(5.99));

        edge->fx = yamlFile.fx;
        edge->fy = yamlFile.fy;
        edge->cx = yamlFile.cx;
        edge->cy = yamlFile.cy;
        edge->Xw[0] = p3d_pose.at<float>(0);
        edge->Xw[1] = p3d_pose.at<float>(1);
        edge->Xw[2] = p3d_pose.at<float>(2);

        optimizer.addEdge(edge);
        VectorEdge.push_back(edge);
    }

    ROS_INFO_STREAM("feature_tracking => optimizer: tracking");
    for(int i0 = 0; i0 < 4; i0++)
    {
        //设置优化参数，开始优化
        optimizer.initializeOptimization(0);
        optimizer.optimize(8);

        for(int j = 0; j < VectorEdge.size(); j++)
        {
            g2o::EdgeSE3ProjectXYZOnlyPose* e = VectorEdge[j];
            MapPoint* pMP = CurrMapPoint[j].first;

            if(e->chi2() > 7.82 || !e->isDepthPositive())
            {
                e->setLevel(1);
                pMP->is_outlier = true;
            }
        }
    }

    cv::Mat matF;
    g2o::VertexSE3Expmap* vF = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
    g2o::SE3Quat se3F = vF->estimate();
    Eigen::Matrix<double, 4, 4> eigenF = se3F.to_homogeneous_matrix();
    cv::eigen2cv(eigenF,matF);
    matF.convertTo(matF,CV_32F);
    CurrFrame->SetPose(matF);

//    ROS_DEBUG_STREAM("*****************************************************");
}