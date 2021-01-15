/*!
 * @file FootSwingTrajectory.h
 * @brief Utility to generate foot swing trajectories.
 * 计算摆动腿的轨迹，为贝塞尔曲线
 *
 * Currently uses Bezier curves like Cheetah 3 does
 */

#ifndef CHEETAH_SOFTWARE_FOOTSWINGTRAJECTORY_H
#define CHEETAH_SOFTWARE_FOOTSWINGTRAJECTORY_H

#include "cppTypes.h"

/*!
 * A foot swing trajectory for a single foot
 * 单腿摆动轨迹的参数
 */
template<typename T>
class FootSwingTrajectory {
public:

  /*!
   * Construct a new foot swing trajectory with everything set to zero
   */
  FootSwingTrajectory() {
    _p0.setZero();
    _pf.setZero();
    _p.setZero();
    _v.setZero();
    _a.setZero();
    _height = 0;
  }

  /*!
   * Set the starting location of the foot
   * @param p0 : the initial foot position，初始位置
   */
  void setInitialPosition(Vec3<T> p0) {
    _p0 = p0;
  }

  /*!
   * Set the desired final position of the foot
   * @param pf : the final foot posiiton，末端位置
   */
  void setFinalPosition(Vec3<T> pf) {
    _pf = pf;
  }

  /*!
   * Set the maximum height of the swing
   * @param h : the maximum height of the swing, achieved halfway through the swing
   * 摆动最大高度
   */
  void setHeight(T h) {
    _height = h;
  }
  /*!
   * 计算贝塞尔摆动轨迹
   */
  void computeSwingTrajectoryBezier(T phase, T swingTime);

  /*!
   * Get the foot position at the current point along the swing
   * @return : the foot position
   */
  Vec3<T> getPosition() {
    return _p;
  }

  /*!
   * Get the foot velocity at the current point along the swing
   * @return : the foot velocity
   */
  Vec3<T> getVelocity() {
    return _v;
  }

  /*!
   * Get the foot acceleration at the current point along the swing
   * @return : the foot acceleration
   */
  Vec3<T> getAcceleration() {
    return _a;
  }

private:
  Vec3<T> _p0, _pf, _p, _v, _a;
  T _height;
};


#endif //CHEETAH_SOFTWARE_FOOTSWINGTRAJECTORY_H
