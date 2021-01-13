/**
 * @file FrameTypedefs.cpp
 * @brief This file contains implementations for MapPoint.
 * @author Jianzhu Huai
 */

#include <okvis/FrameTypedefs.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {

bool MapPoint::goodForMarginalization(size_t minCulledFrames) const {
  if (observations.size() < minCulledFrames)
    return false;
  switch (status.measurementType) {
  case FeatureTrackStatus::kPremature:
    return true;

  case FeatureTrackStatus::kMsckfTrack:
  case FeatureTrackStatus::kSlamInitialization:
    return status.measurementFate != FeatureTrackStatus::kSuccessful;
  default:
    return false;
  }
}

void MapPoint::updateStatus(uint64_t currentFrameId,
                            size_t minTrackLengthForMsckf,
                            size_t minTrackLengthForSlam) {
  bool newlyObserved = trackedInCurrentFrame(currentFrameId);
  status.updateTrackStat(newlyObserved);
  FeatureTrackStatus::MeasurementType measurementType(
      FeatureTrackStatus::kPremature);
  if (newlyObserved) {
    if (status.inState) {
      measurementType = FeatureTrackStatus::kSlamObservation;
    } else {
      if (observations.size() >= minTrackLengthForSlam) {
        measurementType = FeatureTrackStatus::kSlamInitialization;
      }
    }
  } else {
    if (observations.size() >= minTrackLengthForMsckf) {
      measurementType = FeatureTrackStatus::kMsckfTrack;
    }
  }
  status.measurementType = measurementType;
  status.measurementFate = FeatureTrackStatus::kUndetermined;
}

} // namespace okvis
