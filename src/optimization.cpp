#include "optimization.h"

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include <memory>

#include "entities.h"
#include "map.h"
#include "settings.h"

g2o::SE3Quat Converter::toSE3Quat(const cv::Mat &cvMat)
{
  Eigen::Matrix<double, 3, 3> R;
  R << cvMat.at<float>(0, 0), cvMat.at<float>(0, 1), cvMat.at<float>(0, 2),  //
      cvMat.at<float>(1, 0), cvMat.at<float>(1, 1), cvMat.at<float>(1, 2),   //
      cvMat.at<float>(2, 0), cvMat.at<float>(2, 1), cvMat.at<float>(2, 2);

  Eigen::Matrix<double, 3, 1> t(
      cvMat.at<float>(0, 3), cvMat.at<float>(1, 3), cvMat.at<float>(2, 3));

  return g2o::SE3Quat(R, t);
}
Eigen::Matrix<double, 3, 1> Converter::toVector3d(const PosD &pos)
{
  Eigen::Matrix<double, 3, 1> v(pos.x, pos.y, pos.z);
  return v;
}

// =============================================================================

void Optimizer::bundleAdjustment(
    const std::vector<KeyFramePtr> &keyFrames,
    const std::vector<MapPointPtr> &mapPoints,
    int nIterations)
{
  g2o::SparseOptimizer optimizer;
  std::unique_ptr<g2o::BlockSolverX::LinearSolverType> linearSolver =
      std::make_unique<g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>>();

  std::unique_ptr<g2o::BlockSolverX> solver_ptr =
      std::make_unique<g2o::BlockSolverX>(std::move(linearSolver));

  std::unique_ptr<g2o::OptimizationAlgorithmLevenberg> solver =
      std::make_unique<g2o::OptimizationAlgorithmLevenberg>(std::move(solver_ptr));
  optimizer.setAlgorithm(solver.release());

  long unsigned int maxKFid = 0;

  for (size_t i = 0, iend = keyFrames.size(); i < iend; i++)
  {
    KeyFramePtr keyFrame = keyFrames[i];
    // if (keyFrame->isBad()) continue;
    g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
    vSE3->setEstimate(Converter::toSE3Quat(keyFrame->getTwc()));
    vSE3->setId(keyFrame->getID());
    vSE3->setFixed(keyFrame->getID() == 0);
    optimizer.addVertex(vSE3);
    if (keyFrame->getID() > maxKFid) maxKFid = keyFrame->getID();
  }

  // chiThreshold의 제곱근을 사용하는 이유는 huber kernel의 특성때문이다.
  // huber kernel은 오차의 크기에 따라 크면, 1/2*e^2 작으면, |e|의 형태의
  // 오차를 사용한다. 이때, chi-square는 오차의 제곱합 이므로, x^2=e^2이고,
  // e=sqrt(x)가 되기 때문이다.
  const float chiThreshold = sqrt(5.991);

  for (size_t i = 0, iend = mapPoints.size(); i < iend; i++)
  {
    MapPointPtr mapPoint = mapPoints[i];
    // if (mapPoint->isBad()) continue;
    g2o::VertexPointXYZ *vPoint = new g2o::VertexPointXYZ();
    vPoint->setEstimate(Converter::toVector3d(mapPoint->getPos()));
    int id = mapPoint->getID() + maxKFid + 1;
    vPoint->setId(id);
    vPoint->setMarginalized(true);
    optimizer.addVertex(vPoint);

    std::map<KeyFramePtr, ImgPointIdx> observations = mapPoint->getObservations();

    // SET EDGES
    for (std::map<KeyFramePtr, ImgPointIdx>::iterator mit = observations.begin(),
                                                      mend = observations.end();
         mit != mend;
         mit++)
    {
      KeyFramePtr pKF = mit->first;
      // if (pKF->isBad()) continue;
      // Eigen::Matrix<double, 2, 1> obs;
      // cv::KeyPoint kpUn = pKF->GetKeyPointUn(mit->second);
      // obs << kpUn.pt.x, kpUn.pt.y;

      // g2o::EdgeSE3ProjectXYZ *e = new g2o::EdgeSE3ProjectXYZ();

      // e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
      // e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex
      // *>(optimizer.vertex(pKF->mnId))); e->setMeasurement(obs); float invSigma2 =
      // pKF->GetInvSigma2(kpUn.octave); e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

      // g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
      // e->setRobustKernel(rk);
      // rk->setDelta(thHuber);

      // e->fx = pKF->fx;
      // e->fy = pKF->fy;
      // e->cx = pKF->cx;
      // e->cy = pKF->cy;

      // optimizer.addEdge(e);
    }
  }
}