#ifndef INCLUDE_MSCKF_CHECK_SENSOR_RIG_HPP_
#define INCLUDE_MSCKF_CHECK_SENSOR_RIG_HPP_

#include <msckf/imu/ImuRig.hpp>
#include <msckf/CameraRig.hpp>

namespace okvis {
bool doesExtrinsicModelFitImuModel(const std::string& extrinsicModel,
                                   const std::string& imuModel);

bool doesExtrinsicModelFitOkvisBackend(
    const okvis::cameras::NCameraSystem& cameraSystem,
    EstimatorAlgorithm algorithm);
}  // namespace okvis

#endif // INCLUDE_MSCKF_CHECK_SENSOR_RIG_HPP_
