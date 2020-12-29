
/* 该部分是用来计算机体的轨迹 ，主要函数为buildInputTrajectory();*/

#ifndef CHEETAH_SOFTWARE_GRAPHSEARCH_H
#define CHEETAH_SOFTWARE_GRAPHSEARCH_H

#include <vector>
#include "cppTypes.h"


struct ContactState {
  union {
    bool contact[4];
    struct {
      bool fr, fl, rr, rl;
    };
  };

  ContactState(bool _fr, bool _fl, bool _rr, bool _rl) {
    fr = _fr;
    fl = _fl;
    rr = _rr;
    rl = _rl;
  }

  ContactState() { }
};

//默认步态，trot，standing
struct DefaultGaits {
  std::vector<ContactState> trotting, standing;
};
//输入轨迹状态，应该是当前的基座状态，包含p,v,theta
struct InputTrajectoryState {
  Vec2<float> p;
  Vec2<float> v;
  float theta;
};

//足底状态，位置p，接触状态contact，状态时间stateTime
struct FootplanFootState {
  Vec2<float> p;
  bool contact;
  float stateTime;
};

//整体状态，t应该是周期，基坐标位置pBase，四个足底状态feet[4]
struct FootplanState {
  float t;
  Vec2<float> pBase;
  FootplanFootState feet[4];
};

//状态缓冲，nodesVisited,最大容量maxMenory
struct FootplanStats {
  u64 nodesVisited;
  u64 maxMemory;
  
  FootplanStats() {
  //两个参数归零
    reset();
  }

  void reset() {
    nodesVisited = 0;
    maxMemory = 0;
  }
};
//基座期望目标位置
struct FootplanGoal {
  Vec2<float> goalPos;
};

using FootplanStateCost = float (*)(FootplanState&, FootplanGoal&);
using FootplanTransitionCost = float (*)(FootplanState&, FootplanState&, FootplanGoal&);

namespace FootplanCosts {
  //计算当前基座位置到目标基座位置的距离
  float distanceToGoal(FootplanState& state, FootplanGoal& goal);
}

//  cheetah._bodyLength = 0.19 * 2;
//  cheetah._bodyWidth = 0.049 * 2;

class FootstepPlanner {
public:
  FootstepPlanner(bool verbose);
  void reset();
  //根据输入构造轨迹，输入步态周期duration,单位时间dt，输入轨迹状态x0，角速度omega
  void buildInputTrajectory(float duration, float dt, InputTrajectoryState x0, float omega);
  void planFixedEvenGait(std::vector<ContactState>& gait, float gait_period);
  std::vector<InputTrajectoryState>& getInitialTrajectory() {
    return _inputTrajectory;
  }

  void addCost(FootplanStateCost cost) {
    _stateCosts.push_back(cost);
  }

  void addCost(FootplanTransitionCost cost) {
    _transitionCosts.push_back(cost);
  }

  FootplanGoal& getGoal() {
    return _goal;
  }

  DefaultGaits defaults;
private:
  bool _verbose;
  FootplanStats _stats;
  FootplanGoal _goal;

  std::vector<FootplanStateCost> _stateCosts;
  std::vector<FootplanTransitionCost> _transitionCosts;
  std::vector<InputTrajectoryState> _inputTrajectory;
};


#endif //CHEETAH_SOFTWARE_GRAPHSEARCH_H
