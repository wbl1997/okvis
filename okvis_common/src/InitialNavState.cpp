#include "swift_vio/InitialNavState.hpp"

namespace swift_vio {
InitialNavState::InitialNavState()
    : initWithExternalSource(false),
      stateTime(),
      p_WS(0, 0, 0),
      q_WS(1, 0, 0, 0),
      v_WS(0, 0, 0),
      std_p_WS(1e-4, 1e-4, 1e-4),
      std_q_WS(30 * M_PI / 180, 30 * M_PI / 180, 1e-4),
      std_v_WS(0.1, 0.1, 0.1) {}

// v_WS, and std_v_WS are to be recalculated later according to updated p_WS and
// q_ws
InitialNavState::InitialNavState(const InitialNavState& rhs)
    : initWithExternalSource(rhs.initWithExternalSource),
      stateTime(rhs.stateTime),
      p_WS(rhs.p_WS),
      q_WS(rhs.q_WS),
      v_WS(rhs.v_WS),
      std_p_WS(rhs.std_p_WS),
      std_q_WS(rhs.std_q_WS),
      std_v_WS(rhs.std_v_WS) {}

void InitialNavState::updatePose(const okvis::kinematics::Transformation& T_WS,
                                 const okvis::Time state_time) {
  stateTime = state_time;
  p_WS = T_WS.r();
  q_WS = T_WS.q();
}

void InitialNavState::toInformation(
    Eigen::Matrix<double, 6, 6> *information) const {
  information->setZero();
  Eigen::Vector3d positionVariance = std_p_WS.cwiseAbs2();
  Eigen::Vector3d orientationVariance = std_q_WS.cwiseAbs2();
  information->diagonal().head<3>() = positionVariance.cwiseInverse();
  information->diagonal().tail<3>() = orientationVariance.cwiseInverse();
}

void InitialNavState::toCovariance(
    Eigen::Matrix<double, 6, 6> *covariance) const {
  covariance->setZero();
  covariance->diagonal().head<3>() = std_p_WS.cwiseAbs2();
  covariance->diagonal().tail<3>() = std_q_WS.cwiseAbs2();
}

InitialNavState& InitialNavState::operator=(const InitialNavState& other) {
  if (&other == this) return *this;
  initWithExternalSource = other.initWithExternalSource;
  stateTime = other.stateTime;
  p_WS = other.p_WS;
  q_WS = other.q_WS;
  v_WS = other.v_WS;
  std_p_WS = other.std_p_WS;
  std_q_WS = other.std_q_WS;
  std_v_WS = other.std_v_WS;
  return *this;
}
}  // namespace swift_vio
