
/**
 * @file implementation/Estimator.hpp
 * @brief Header implementation file for the Estimator class.
 * @author Jianzhu Huai
 */

#include <msckf/EpipolarFactor.hpp>
#include <msckf/ProjParamOptModels.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {

template<class GEOMETRY_TYPE>
::ceres::ResidualBlockId Estimator::addPointFrameResidual(
    uint64_t landmarkId,
    uint64_t poseId,
    size_t camIdx,
    const Eigen::Vector2d& measurement,
    const Eigen::Matrix2d& information,
    std::shared_ptr<const GEOMETRY_TYPE> cameraGeometry) {
  // TODO(jhuai): add residual according to
  // (camera_rig_.getProjectionOptMode(camIdx) == ProjectionOptFixed::kModelId)
  std::shared_ptr < ceres::ReprojectionError
      < GEOMETRY_TYPE
          >> reprojectionError(
              new ceres::ReprojectionError<GEOMETRY_TYPE>(
                  cameraGeometry,
                  camIdx, measurement, information));

  ::ceres::ResidualBlockId retVal = mapPtr_->addResidualBlock(
      reprojectionError,
      cauchyLossFunctionPtr_ ? cauchyLossFunctionPtr_.get() : NULL,
      mapPtr_->parameterBlockPtr(poseId),
      mapPtr_->parameterBlockPtr(landmarkId),
      mapPtr_->parameterBlockPtr(
          statesMap_.at(poseId).sensors.at(SensorStates::Camera).at(camIdx).at(
              CameraSensorStates::T_SCi).id));
  return retVal;
}

template<class PARAMETER_BLOCK_T>
bool Estimator::getSensorStateEstimateAs(
    uint64_t poseId, int sensorIdx, int sensorType, int stateType,
    typename PARAMETER_BLOCK_T::estimate_t & state) const
{
#if 0
  PARAMETER_BLOCK_T stateParameterBlock;
  if (!getSensorStateParameterBlockAs(poseId, sensorIdx, sensorType, stateType,
                                      stateParameterBlock)) {
    return false;
  }
  state = stateParameterBlock.estimate();
  return true;
#else
  // convert base class pointer with various levels of checking
  std::shared_ptr<ceres::ParameterBlock> parameterBlockPtr;
  if (!getSensorStateParameterBlockPtr(poseId, sensorIdx, sensorType, stateType,
                                       parameterBlockPtr)) {
      return false;
  }
#ifndef NDEBUG
  std::shared_ptr<PARAMETER_BLOCK_T> derivedParameterBlockPtr =
          std::dynamic_pointer_cast<PARAMETER_BLOCK_T>(parameterBlockPtr);
  if(!derivedParameterBlockPtr) {
      std::shared_ptr<PARAMETER_BLOCK_T> info(new PARAMETER_BLOCK_T);
      OKVIS_THROW_DBG(Exception,"wrong pointer type requested: requested "
                      <<info->typeInfo()<<" but is of type"
                      <<parameterBlockPtr->typeInfo())
              return false;
  }
  state = derivedParameterBlockPtr->estimate();
#else
  state = std::static_pointer_cast<PARAMETER_BLOCK_T>(
              parameterBlockPtr)->estimate();
#endif
  return true;
#endif
}

template <class CAMERA_GEOMETRY_T>
bool Estimator::replaceEpipolarWithReprojectionErrors(uint64_t lmId) {
  PointMap::iterator it = landmarksMap_.find(lmId);
  std::map<okvis::KeypointIdentifier, uint64_t>& obsMap = it->second.observations;
  // remove all previous (epipolar constraint) residual blocks for
  // this landmark if exist, use the ResidualBlockId which is the map value

  // add all observations as reprojection errors
  return true;
}

template <class CAMERA_GEOMETRY_T>
bool Estimator::addEpipolarConstraint(uint64_t landmarkId, uint64_t poseId,
                                      size_t camIdx, size_t keypointIdx,
                                      bool removeExisting) {
  PointMap::iterator lmkIt = landmarksMap_.find(landmarkId);
  if (lmkIt == landmarksMap_.end())
    return false;
  okvis::KeypointIdentifier kidTail(poseId, camIdx, keypointIdx);

  // avoid double observations
  if (lmkIt->second.observations.find(kidTail) !=
      lmkIt->second.observations.end()) {
    return false;
  }

  std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>
      measurement12(2);
  std::vector<Eigen::Matrix2d, Eigen::aligned_allocator<Eigen::Matrix2d>>
      covariance12(2);
  // get the head keypoint measurement
  const okvis::KeypointIdentifier& kidHead =
      lmkIt->second.observations.begin()->first;
  okvis::MultiFramePtr multiFramePtr =
      multiFramePtrMap_.at(kidHead.frameId);
  multiFramePtr->getKeypoint(kidHead.cameraIndex, kidHead.keypointIndex,
                             measurement12[0]);
  covariance12[0] = Eigen::Matrix2d::Identity();
  double size = 1.0;
  multiFramePtr->getKeypointSize(kidHead.cameraIndex,
                                 kidHead.keypointIndex, size);
  covariance12[0] *= (size * size) / 64.0;

  // get the tail keypoint measurement
  multiFramePtr = multiFramePtrMap_.at(poseId);
  multiFramePtr->getKeypoint(camIdx, keypointIdx, measurement12[1]);
  covariance12[1] = Eigen::Matrix2d::Identity();
  size = 1.0;
  multiFramePtr->getKeypointSize(camIdx, keypointIdx, size);
  covariance12[1] *= (size * size) / 64.0;

  std::shared_ptr<okvis::cameras::CameraBase> baseCameraGeometry =
      camera_rig_.getCameraGeometry(camIdx);
  std::shared_ptr<const CAMERA_GEOMETRY_T> argCameraGeometry =
      std::static_pointer_cast<const CAMERA_GEOMETRY_T>(baseCameraGeometry);

  auto& stateLeft = statesMap_.at(kidHead.frameId);
  auto& stateRight = statesMap_.at(poseId);

  std::vector<okvis::Time> stateEpoch = {stateLeft.timestamp,
                                         stateRight.timestamp};
  std::vector<okvis::ImuMeasurementDeque,
              Eigen::aligned_allocator<okvis::ImuMeasurementDeque>>
      imuMeasCanopy = {
          inertialMeasForStates_.findWindow(stateEpoch[0], half_window_),
          inertialMeasForStates_.findWindow(stateEpoch[1], half_window_)};
  okvis::kinematics::Transformation T_SC_base =
      camera_rig_.getCameraExtrinsic(camIdx);

  std::vector<double> tdAtCreation = {stateLeft.tdAtCreation.toSec(),
                                      stateRight.tdAtCreation.toSec()};
  double gravityMag = imuParametersVec_.at(0).g;

  std::shared_ptr<ceres::EpipolarFactor<CAMERA_GEOMETRY_T, Extrinsic_p_SC_q_SC,
                                        ProjectionOptFXY_CXY>>
      twoViewError(
          new ceres::EpipolarFactor<CAMERA_GEOMETRY_T, Extrinsic_p_SC_q_SC,
                                    ProjectionOptFXY_CXY>(
              argCameraGeometry, landmarkId, measurement12, covariance12,
              imuMeasCanopy, T_SC_base, stateEpoch, tdAtCreation, gravityMag));

  ::ceres::ResidualBlockId retVal = mapPtr_->addResidualBlock(
      twoViewError,
      cauchyLossFunctionPtr_ ? cauchyLossFunctionPtr_.get() : NULL,
      mapPtr_->parameterBlockPtr(kidHead.frameId),
      mapPtr_->parameterBlockPtr(poseId),
      mapPtr_->parameterBlockPtr(stateRight.sensors.at(SensorStates::Camera)
                                     .at(camIdx)
                                     .at(CameraSensorStates::T_SCi)
                                     .id),
      mapPtr_->parameterBlockPtr(stateRight.sensors.at(SensorStates::Camera)
                                     .at(camIdx)
                                     .at(CameraSensorStates::Intrinsics)
                                     .id),
      mapPtr_->parameterBlockPtr(stateRight.sensors.at(SensorStates::Camera)
                                     .at(camIdx)
                                     .at(CameraSensorStates::Distortion)
                                     .id),
      mapPtr_->parameterBlockPtr(stateRight.sensors.at(SensorStates::Camera)
                                     .at(camIdx)
                                     .at(CameraSensorStates::TR)
                                     .id),
      mapPtr_->parameterBlockPtr(stateRight.sensors.at(SensorStates::Camera)
                                     .at(camIdx)
                                     .at(CameraSensorStates::TD)
                                     .id),
      mapPtr_->parameterBlockPtr(stateLeft.sensors.at(SensorStates::Imu)
                                     .at(0)
                                     .at(ImuSensorStates::SpeedAndBias)
                                     .id),
      mapPtr_->parameterBlockPtr(stateRight.sensors.at(SensorStates::Imu)
                                     .at(0)
                                     .at(ImuSensorStates::SpeedAndBias)
                                     .id));

  if (removeExisting) {
    for (auto obsIter = lmkIt->second.observations.begin();
         obsIter != lmkIt->second.observations.end(); ++obsIter) {
      if (obsIter->second != 0) {
        mapPtr_->removeResidualBlock(
            reinterpret_cast<::ceres::ResidualBlockId>(obsIter->second));
      }
    }
  }

  // remember
  lmkIt->second.observations.insert(
      std::pair<okvis::KeypointIdentifier, uint64_t>(
          kidTail, reinterpret_cast<uint64_t>(retVal)));

  return true;
}
}  // namespace okvis
