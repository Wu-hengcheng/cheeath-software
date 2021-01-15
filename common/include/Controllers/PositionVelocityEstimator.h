/*! @file PositionVelocityEstimator.h
 *  @brief All State Estimation Algorithms
 *
 *  This file will contain all state estimation algorithms.
 *  PositionVelocityEstimators should compute:
 *  - body position/velocity in world/body frames
 *  - foot positions/velocities in body/world frame
 *  位置速度估计器需要计算：机体和足部在世界/本地坐标系下的位置和速度
 */

#ifndef PROJECT_POSITIONVELOCITYESTIMATOR_H
#define PROJECT_POSITIONVELOCITYESTIMATOR_H

#include "Controllers/StateEstimatorContainer.h"

/*!
 * Position and velocity estimator based on a Kalman Filter.
 * This is the algorithm used in Mini Cheetah and Cheetah 3.
 */
template <typename T>
class LinearKFPositionVelocityEstimator : public GenericEstimator<T> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LinearKFPositionVelocityEstimator();
  virtual void run();
  virtual void setup();

 private:
  Eigen::Matrix<T, 18, 1> _xhat;
  Eigen::Matrix<T, 12, 1> _ps;
  Eigen::Matrix<T, 12, 1> _vs;
  Eigen::Matrix<T, 18, 18> _A;
  Eigen::Matrix<T, 18, 18> _Q0;
  Eigen::Matrix<T, 18, 18> _P;
  Eigen::Matrix<T, 28, 28> _R0;
  Eigen::Matrix<T, 18, 3> _B;
  Eigen::Matrix<T, 28, 18> _C;
};

/*!
 * "Cheater" position and velocity estimator which will return the correct position and
 * velocity when running in simulation.
 */
template<typename T>
class CheaterPositionVelocityEstimator : public GenericEstimator<T> {
public:
  virtual void run();
  virtual void setup() {}
};

#endif  // PROJECT_POSITIONVELOCITYESTIMATOR_H
