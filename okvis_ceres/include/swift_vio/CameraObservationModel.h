#ifndef SWIFT_VIO_CAMERA_OBSERVATION_MODEL_H
#define SWIFT_VIO_CAMERA_OBSERVATION_MODEL_H

namespace swift_vio {
class CameraObservationModel {
public:
  static int toResidualDim(int modelId);
};

#ifndef RESIDUAL_MODEL_SWITCH_CASES
#define RESIDUAL_MODEL_SWITCH_CASES                                            \
  RESIDUAL_MODEL_CASE(okvis::ceres::ReprojectionError2dBase)                   \
  RESIDUAL_MODEL_CASE(okvis::ceres::EpipolarFactorBase)                        \
  RESIDUAL_MODEL_CASE(okvis::ceres::ChordalDistanceBase)                       \
  RESIDUAL_MODEL_CASE(okvis::ceres::ReprojectionErrorWithPapBase)              \
  RESIDUAL_MODEL_CASE(okvis::ceres::RsReprojectionErrorBase)                   \
  RESIDUAL_MODEL_CASE(okvis::ceres::RSCameraReprojectionErrorBase)             \
  default:                                                                     \
    MODEL_DOES_NOT_EXIST_EXCEPTION                                             \
    break;
#endif

} // namespace swift_vio

#endif // SWIFT_VIO_CAMERA_OBSERVATION_MODEL_H
