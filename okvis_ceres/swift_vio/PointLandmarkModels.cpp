#include "swift_vio/PointLandmarkModels.hpp"
#include <swift_vio/ParallaxAnglePoint.hpp>
#include <okvis/ceres/HomogeneousPointLocalParameterization.hpp>


namespace swift_vio {
bool ParallaxAngleParameterization::plus(const double *x, const double *delta,
                                         double *x_plus_delta) {
  Eigen::Map<const Eigen::Vector3d> _delta(delta);
  swift_vio::ParallaxAnglePoint pap(x[3], x[0], x[1], x[2], x[4], x[5]);
  pap.boxPlus(_delta, pap);

  const double *bearingData = pap.n_.data();
  x_plus_delta[0] = bearingData[0];
  x_plus_delta[1] = bearingData[1];
  x_plus_delta[2] = bearingData[2];
  x_plus_delta[3] = bearingData[3];
  const double *thetaData = pap.theta_.data();
  x_plus_delta[4] = thetaData[0];
  x_plus_delta[5] = thetaData[1];
  return true;
}

bool ParallaxAngleParameterization::minus(const double* /*x*/, const double* /*x_plus_delta*/, double* /*delta*/) {
  OKVIS_ASSERT_TRUE(std::runtime_error, false, "Not implemented!");
  return false;
}

bool ParallaxAngleParameterization::plusJacobian(const double*, double* /*jacobian*/) {
  OKVIS_ASSERT_TRUE(std::runtime_error, false, "Not implemented!");
  return false;
}

bool InverseDepthParameterization::plus(const double *x, const double *delta,
                                        double *x_plus_delta) {
  x_plus_delta[0] = x[0] + delta[0];
  x_plus_delta[1] = x[1] + delta[1];
  x_plus_delta[2] = x[2];
  x_plus_delta[3] = x[3] + delta[2];
  return true;
}

bool InverseDepthParameterization::minus(const double* x,
                                         const double* x_plus_delta,
                                         double* delta) {
  delta[0] = x_plus_delta[0] - x[0];
  delta[1] = x_plus_delta[1] - x[1];
  delta[2] = x_plus_delta[3] - x[3];
  return true;
}

std::shared_ptr<okvis::ceres::LocalParamizationAdditionalInterfaces> createLandmarkLocalParameterization(int modelId) {
  std::shared_ptr<okvis::ceres::LocalParamizationAdditionalInterfaces> parameterizationPtr;
  switch (modelId) {
    case okvis::ceres::HomogeneousPointLocalParameterization::kModelId:
      parameterizationPtr.reset(new okvis::ceres::HomogeneousPointLocalParameterization());
      break;
    case InverseDepthParameterization::kModelId:
      parameterizationPtr.reset(new InverseDepthParameterization());
      break;
    case ParallaxAngleParameterization::kModelId:
      parameterizationPtr.reset(new ParallaxAngleParameterization());
      break;
  }
  return parameterizationPtr;
}

} // namespace swift_vio
