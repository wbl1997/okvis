#include <swift_vio/SwiftParameters.hpp>
#include <algorithm>
#include <sstream>
#include <unordered_map>

namespace swift_vio {
EstimatorAlgorithm EstimatorAlgorithmNameToId(std::string description) {
  std::transform(description.begin(), description.end(), description.begin(),
                 ::toupper);
  std::unordered_map<std::string, EstimatorAlgorithm> descriptionToId{
      {"OKVIS", EstimatorAlgorithm::OKVIS},
      {"GENERAL", EstimatorAlgorithm::General},
      {"MSCKF", EstimatorAlgorithm::MSCKF},
      {"TFVIO", EstimatorAlgorithm::TFVIO},
      {"INVARIANTEKF", EstimatorAlgorithm::InvariantEKF},
      {"SLIDINGWINDOWSMOOTHER", EstimatorAlgorithm::SlidingWindowSmoother},
      {"RISLIDINGWINDOWSMOOTHER", EstimatorAlgorithm::RiSlidingWindowSmoother},
      {"HYBRIDFILTER", EstimatorAlgorithm::HybridFilter},
  };

  auto iter = descriptionToId.find(description);
  if (iter == descriptionToId.end()) {
    return EstimatorAlgorithm::OKVIS;
  } else {
    return iter->second;
  }
}

struct EstimatorAlgorithmHash {
  template <typename T>
  std::size_t operator()(T t) const {
    return static_cast<std::size_t>(t);
  }
};

std::string EstimatorAlgorithmIdToName(EstimatorAlgorithm id) {
  std::unordered_map<EstimatorAlgorithm, std::string, EstimatorAlgorithmHash>
      idToDescription{
          {EstimatorAlgorithm::OKVIS, "OKVIS"},
          {EstimatorAlgorithm::General, "General"},
          {EstimatorAlgorithm::MSCKF, "MSCKF"},
          {EstimatorAlgorithm::TFVIO, "TFVIO"},
          {EstimatorAlgorithm::InvariantEKF, "InvariantEKF"},
          {EstimatorAlgorithm::SlidingWindowSmoother, "SlidingWindowSmoother"},
          {EstimatorAlgorithm::RiSlidingWindowSmoother,
           "RiSlidingWindowSmoother"},
          {EstimatorAlgorithm::HybridFilter,
           "HybridFilter"}};
  auto iter = idToDescription.find(id);
  if (iter == idToDescription.end()) {
    return "OKVIS";
  } else {
    return iter->second;
  }
}

FrontendOptions::FrontendOptions(bool initWithoutEnoughParallax,
                                 bool stereoWithEpipolarCheck,
                                 double epipolarDistanceThresh,
                                 int featureTrackingApproach)
    : initializeWithoutEnoughParallax(initWithoutEnoughParallax),
      stereoMatchWithEpipolarCheck(stereoWithEpipolarCheck),
      epipolarDistanceThreshold(epipolarDistanceThresh),
      featureTrackingMethod(featureTrackingApproach) {}

PoseGraphOptions::PoseGraphOptions()
    : maxOdometryConstraintForAKeyframe(3), minDistance(0.1), minAngle(0.1) {}

PointLandmarkOptions::PointLandmarkOptions()
    : landmarkModelId(0), minTrackLengthForMsckf(3u),
      anchorAtObservationTime(false), maxHibernationFrames(3u),
      minTrackLengthForSlam(11u), maxInStateLandmarks(50),
      maxMarginalizedLandmarks(50) {}

PointLandmarkOptions::PointLandmarkOptions(
    int lmkModelId, size_t minMsckfTrackLength, bool anchorAtObsTime,
    size_t hibernationFrames, size_t minSlamTrackLength, int maxStateLandmarks,
    int maxMargedLandmarks)
    : landmarkModelId(lmkModelId), minTrackLengthForMsckf(minMsckfTrackLength),
      anchorAtObservationTime(anchorAtObsTime),
      maxHibernationFrames(hibernationFrames),
      minTrackLengthForSlam(minSlamTrackLength),
      maxInStateLandmarks(maxStateLandmarks),
      maxMarginalizedLandmarks(maxMargedLandmarks) {}

std::string PointLandmarkOptions::toString(std::string lead) const {
  std::stringstream ss(lead);
  ss << "Landmark model id " << landmarkModelId
     << " anchor at observation epoch (state epoch) ? "
     << anchorAtObservationTime << "\n#hibernation frames "
     << maxHibernationFrames << " track length for MSCKF "
     << minTrackLengthForMsckf << " for SLAM " << minTrackLengthForSlam
     << ". Max landmarks in state " << maxInStateLandmarks
     << ", max landmarks marginalized in one update step "
     << maxMarginalizedLandmarks << ".";
  return ss.str();
}
}  // namespace swift_vio
