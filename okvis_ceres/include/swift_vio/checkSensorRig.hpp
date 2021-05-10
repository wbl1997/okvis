#ifndef INCLUDE_SWIFT_VIO_CHECK_SENSOR_RIG_HPP_
#define INCLUDE_SWIFT_VIO_CHECK_SENSOR_RIG_HPP_

#include <swift_vio/imu/ImuRig.hpp>
#include <swift_vio/CameraRig.hpp>

namespace okvis {
bool doesExtrinsicModelFitImuModel(const std::string& extrinsicModel,
                                   const std::string& imuModel);

bool doesExtrinsicModelFitOkvisBackend(
    const okvis::cameras::NCameraSystem& cameraSystem,
    EstimatorAlgorithm algorithm);
}  // namespace okvis

#endif // INCLUDE_SWIFT_VIO_CHECK_SENSOR_RIG_HPP_
