#include "optimization.h"

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include <opencv2/core/traits.hpp>
// #include <g2o/types/sba/edge_project_xyz.h>
// #include <g2o/types/sba/vertex_se3_expmap.h>
// #include <g2o/types/slam3d/vertex_pointxyz.h>

// #include "entities.h"
#include "entities.h"
#include "frame.h"
#include "map.h"
#include "misc.h"

constexpr float ChiSquareTh2D = 5.99;
constexpr float ChiSquareTh3D = 7.815;
// chiThreshold의 제곱근을 사용하는 이유는 huber kernel의 특성때문이다.
// huber kernel은 오차의 크기에 따라 크면, 1/2*e^2 작으면, |e|의 형태의
// 오차를 사용한다. 이때, chi-square는 오차의 제곱합 이므로, x^2=e^2이고,
// e=sqrt(x)가 되기 때문이다.

g2o::SE3Quat Converter::ToSE3Quat(const cv::Mat &cvMat)
{
  Eigen::Matrix<double, 3, 3> R;
  R << cvMat.at<double>(0, 0), cvMat.at<double>(0, 1),
      cvMat.at<double>(0, 2),  //
      cvMat.at<double>(1, 0), cvMat.at<double>(1, 1),
      cvMat.at<double>(1, 2),  //
      cvMat.at<double>(2, 0), cvMat.at<double>(2, 1), cvMat.at<double>(2, 2);

  Eigen::Matrix<double, 3, 1> t(
      cvMat.at<double>(0, 3), cvMat.at<double>(1, 3), cvMat.at<double>(2, 3));

  return g2o::SE3Quat(R, t);
}
cv::Mat Converter::ToCvMat(const g2o::SE3Quat input)
{
  Eigen::Matrix<double, 4, 4> eigMat = input.to_homogeneous_matrix();
  return ToCvMat(eigMat);
}
cv::Mat Converter::ToCvMat(const Eigen::Matrix<double, 4, 4> &input)
{
  cv::Mat cvMat(4, 4, CV_64F);
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 4; j++)
    {
      cvMat.at<double>(i, j) = input(i, j);
    }
  }
  return cvMat.clone();
}
cv::Mat Converter::ToCvMat(const Eigen::Matrix<double, 3, 1> &m)
{
  cv::Mat cvMat(3, 1, CV_64F);
  for (int i = 0; i < 3; i++)
  {
    cvMat.at<double>(i) = m(i);
  }
  return cvMat.clone();
}
Eigen::Matrix<double, 3, 1> Converter::ToVector3d(const PosD &pos)
{
  Eigen::Matrix<double, 3, 1> v(pos.x, pos.y, pos.z);
  return v;
}
Eigen::Matrix<double, 3, 1> Converter::ToVector3d(const cv::Mat &pos)
{
  Eigen::Matrix<double, 3, 1> v(
      pos.at<double>(0), pos.at<double>(1), pos.at<double>(2));
  return v;
}

// =============================================================================
void Optimizer::GlobalBundleAdjustment(Map *pMap, uint nIterators)
{
  std::vector<KeyFrame *> vpKFs = pMap->getAllKFs();
  std::vector<MapPoint *> vpMPs = pMap->getAllMPs();
  Optimizer::BundleAdjustment(vpKFs, vpMPs, nIterators);
}

void Optimizer::BundleAdjustment(
    const std::vector<KeyFrame *> &vpKFs,
    const std::vector<MapPoint *> &vpMPs,
    int nIterations)
{
  // std::unique_ptr<g2o::BlockSolverX::LinearSolverType> linearSolver =
  //     std::make_unique<
  //         g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>>();
  std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver =
      std::make_unique<
          g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>();
  std::unique_ptr<g2o::BlockSolver_6_3> pSolver =
      std::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver));

  // std::unique_ptr<g2o::BlockSolverX> solver_ptr =
  //     std::make_unique<g2o::BlockSolverX>(std::move(linearSolver));

  // std::unique_ptr<g2o::OptimizationAlgorithmLevenberg> solver =
  // std::make_unique<g2o::OptimizationAlgorithmLevenberg>(std::move(pSolver));
  // optimizer.setAlgorithm(solver.release());

  g2o::OptimizationAlgorithmLevenberg *solver =
      new g2o::OptimizationAlgorithmLevenberg(std::move(pSolver));
  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(solver);

  long unsigned int maxKFID = 0;
  for (size_t i = 0; i < vpKFs.size(); ++i)
  {
    KeyFrame *pKF = vpKFs[i];
    // if (keyFrame->isBad()) continue;
    g2o::VertexSE3Expmap *vertexKF = new g2o::VertexSE3Expmap();
    vertexKF->setEstimate(Converter::ToSE3Quat(pKF->getPose()));
    vertexKF->setId(pKF->getID());
    vertexKF->setFixed(pKF->getID() == 0);
    optimizer.addVertex(vertexKF);
    if (pKF->getID() > maxKFID)
    {
      maxKFID = pKF->getID();
    }
  }

  for (size_t i = 0; i < vpMPs.size(); ++i)
  {
    MapPoint *pMP = vpMPs[i];
    // if (mapPoint->isBad()) continue;
    g2o::VertexPointXYZ *vertexMP = new g2o::VertexPointXYZ();
    vertexMP->setEstimate(Converter::ToVector3d(pMP->getPos()));
    //   vPoint->setEstimate(mapPoint->getPosEigen());
    const int mpID = pMP->getID() + maxKFID + 1;
    vertexMP->setId(mpID);
    vertexMP->setMarginalized(true);
    optimizer.addVertex(vertexMP);

    std::map<KeyFramePtr, uvIdx> observations = pMP->getObservations();

    // SET EDGES
    for (const auto &[pKF, imgIdx] : observations)
    {
      // KeyFramePtr pKF = mit->first;
      // if (pKF->isBad()) continue;
      Eigen::Matrix<double, 2, 1> obs;
      const cv::KeyPoint kp = pKF->getKps()[imgIdx];
      obs << kp.pt.x, kp.pt.y;

      g2o::EdgeSE3ProjectXYZ *e = new g2o::EdgeSE3ProjectXYZ();
      e->setVertex(
          0,
          dynamic_cast<g2o::OptimizableGraph::Vertex *>(
              optimizer.vertex(mpID)));
      e->setVertex(
          1,
          dynamic_cast<g2o::OptimizableGraph::Vertex *>(
              optimizer.vertex(pKF->getID())));
      e->setMeasurement(obs);
      // float invSigma2 = pKF->getInvScaleFactors2(kp.octave);
      float invSigma2 = pKF->invScaleFactors2[kp.octave];
      e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

      // if(rubust)
      // g2o::RobustKernelHuber *rubustKernel = new g2o::RobustKernelHuber;
      // rubustKernel->setDelta(ChiSquareTh2D);
      // e->setRobustKernel(rubustKernel);

      e->fx = pKF->fx;
      e->fy = pKF->fy;
      e->cx = pKF->cx;
      e->cy = pKF->cy;

      optimizer.addEdge(e);
    }
  }

  optimizer.initializeOptimization();
  optimizer.optimize(nIterations);

  // Restore new the pose of Keyframes
  for (size_t i = 0; i < vpKFs.size(); i++)
  {
    KeyFrame *pKF = vpKFs[i];
    // if (pKF->isBad()) continue;
    g2o::VertexSE3Expmap *vertexPose =
        static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(pKF->getID()));
    g2o::SE3Quat SE3quat = vertexPose->estimate();
    PRINT(SE3quat.to_homogeneous_matrix());
    PRINT(Converter::ToCvMat(SE3quat));
    // if (nLoopKF == 0)
    // {
    pKF->setPose(Converter::ToCvMat(SE3quat));
    // }
    // else
    // {
    //   pKF->mTcwGBA.create(4, 4, CV_32F);
    //   Converter::toCvMat(SE3quat).copyTo(pKF->mTcwGBA);
    //   pKF->mnBAGlobalForKF = nLoopKF;
    // }
  }

  // Points
  for (size_t i = 0; i < vpMPs.size(); i++)
  {
    // if (vbNotIncludedMP[i]) continue;
    MapPoint *pMP = vpMPs[i];
    // if (pMP->isBad()) continue;
    g2o::VertexPointXYZ *vertexMapPoint = static_cast<g2o::VertexPointXYZ *>(
        optimizer.vertex(pMP->getID() + maxKFID + 1));

    // if (nLoopKF == 0)
    // {
    pMP->setPos(Converter::ToCvMat(vertexMapPoint->estimate()));
    pMP->updateNormalAndDepth();
    // }
    // else
    // {
    //   pMP->mPosGBA.create(3, 1, CV_32F);
    //   Converter::toCvMat(vPoint->estimate()).copyTo(pMP->mPosGBA);
    //   pMP->mnBAGlobalForKF = nLoopKF;
    // }
  }
}