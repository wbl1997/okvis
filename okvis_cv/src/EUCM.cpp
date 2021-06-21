
/**
 * @file EUCM.cpp
 * @brief Source file for the extended unified camera model class.
 * @author Jianzhu Huai
 */

#include <okvis/cameras/EUCM.hpp>
#include <okvis/cameras/extended_camera.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {
/// \brief cameras Namespace for camera-related functionality.
namespace cameras {

EUCM::EUCM(int imageWidth, int imageHeight, double focalLengthU,
           double focalLengthV, double imageCenterU, double imageCenterV,
           double alpha, double beta, double delayTime, double readoutTime,
           uint64_t id)
    : CameraBase(imageWidth, imageHeight, delayTime, readoutTime, id) {
  Eigen::Matrix<double, NumIntrinsics, 1> intrinsics;
  intrinsics[0] = focalLengthU; //< focalLengthU
  intrinsics[1] = focalLengthV; //< focalLengthV
  intrinsics[2] = imageCenterU; //< imageCenterU
  intrinsics[3] = imageCenterV; //< imageCenterV
  intrinsics[4] = alpha;
  intrinsics[5] = beta;
  eucm_ = ExtendedUnifiedCamera<double>(intrinsics);
}

void EUCM::getIntrinsics(Eigen::VectorXd &intrinsics) const {
  intrinsics = eucm_.getParam();
}

bool EUCM::setIntrinsics(const Eigen::VectorXd &intrinsics) {
  if (intrinsics.size() != NumIntrinsics) {
    return false;
  }
  eucm_ = ExtendedUnifiedCamera<double>(intrinsics);
  return true;
}

CameraBase::ProjectionStatus EUCM::project(const Eigen::Vector3d &point,
                                           Eigen::Vector2d *imagePoint) const {
  bool status = eucm_.project(point, *imagePoint);
  if (status) {
    return okvis::cameras::CameraBase::ProjectionStatus::Successful;
  }
  return okvis::cameras::CameraBase::ProjectionStatus::Invalid;
}

CameraBase::ProjectionStatus
EUCM::project(const Eigen::Vector3d &point, Eigen::Vector2d *imagePoint,
              Eigen::Matrix<double, 2, 3> *pointJacobian,
              Eigen::Matrix2Xd *intrinsicsJacobian) const {
  intrinsicsJacobian->resize(2, NumIntrinsics);
  bool status =
      eucm_.project(point, *imagePoint, pointJacobian, intrinsicsJacobian);
  if (status) {
    return okvis::cameras::CameraBase::ProjectionStatus::Successful;
  }
  return okvis::cameras::CameraBase::ProjectionStatus::Invalid;
}

CameraBase::ProjectionStatus EUCM::projectWithExternalParameters(
    const Eigen::Vector3d &point, const Eigen::VectorXd &parameters,
    Eigen::Vector2d *imagePoint, Eigen::Matrix<double, 2, 3> */*pointJacobian*/,
    Eigen::Matrix2Xd */*intrinsicsJacobian*/) const {
  ExtendedUnifiedCamera<double> eucm(parameters);
  bool status = eucm.project(point, *imagePoint);
  if (status) {
    return okvis::cameras::CameraBase::ProjectionStatus::Successful;
  }
  return okvis::cameras::CameraBase::ProjectionStatus::Invalid;
}

bool EUCM::backProject(const Eigen::Vector2d &imagePoint,
                       Eigen::Vector3d *direction) const {
  return eucm_.unproject(imagePoint, *direction);
}
} // namespace cameras
} // namespace okvis
