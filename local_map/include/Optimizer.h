//
// Created by zhangzuo on 22-8-30.
//

#ifndef SRC_OPTIMIZER_H
#define SRC_OPTIMIZER_H

#include <iostream>
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/solvers/structure_only/structure_only_solver.h"
#include <opencv2/core/eigen.hpp>
#include "Eigen/Core"
#include "sophus/se3.hpp"
#include <Eigen/StdVector>
#include "LocalMap.h"
#include "KeyFrame.h"
#include "MapPoint.h"


void LocalMapBA(vector<KeyFrame*> &LocalKeyFrames, KeyFrame* CurrKeyFrame, vector<KeyFrame*> &NewPartKeyFrame, vector<MapPoint*> &NewMapPoints, cv::Mat &K);

void LocalMapG2O(vector<KeyFrame*> &mOptKeyFrame, vector<KeyFrame*> &mFixKeyFrame, vector<MapPoint*> &mOptMapPoints, cv::Mat &K);

g2o::SE3Quat Mat2SE3Quat(cv::Mat &m2sTcw);


#endif //SRC_OPTIMIZER_H
