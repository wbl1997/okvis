/*********************************************************************************
 *  OKVIS - Open Keyframe-based Visual-Inertial SLAM
 *  Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 * 
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *   * Neither the name of Autonomous Systems Lab / ETH Zurich nor the names of
 *     its contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Created on: Mar 27, 2013
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *    Modified: Andreas Forster (an.forster@gmail.com)
 *********************************************************************************/

/**
 * @file FrameTypedefs.hpp
 * @brief This file contains useful typedefs and structs related to frames.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#ifndef INCLUDE_OKVIS_FRAMETYPEDEFS_HPP_
#define INCLUDE_OKVIS_FRAMETYPEDEFS_HPP_

#include <map>

#include <Eigen/Core>
#include <okvis/kinematics/Transformation.hpp>
#include <swift_vio/FrameTypedefs.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {

/**
 * \brief Unique identifier for a keypoint.
 *
 * A keypoint is identified as the keypoint with index \e keypointIndex
 * in the frame with index \e cameraIndex of multiframe with ID \e frameID.
 */
struct KeypointIdentifier
{
  /**
   * @brief Constructor.
   * @param fi Multiframe ID.
   * @param ci Camera index.
   * @param ki Keypoint index.
   */
  KeypointIdentifier(uint64_t fi = 0, size_t ci = 0, size_t ki = 0)
      : frameId(fi),
        cameraIndex(ci),
        keypointIndex(ki)
  {
  }

  uint64_t frameId;     ///< Multiframe ID.
  size_t cameraIndex;   ///< Camera index.
  size_t keypointIndex; ///< Index of the keypoint

  /// \brief Get multiframe ID.
  uint64_t getFrameId()
  {
    return frameId;
  }
  /// \brief Set multiframe ID.
  void setFrameId(uint64_t fid)
  {
    frameId = fid;
  }
  /// \brief Are two identifiers identical?
  bool isBinaryEqual(const KeypointIdentifier & rhs) const
  {
    return frameId == rhs.frameId && cameraIndex == rhs.cameraIndex
        && keypointIndex == rhs.keypointIndex;
  }
  /// \brief Equal to operator.
  bool operator==(const KeypointIdentifier & rhs) const
  {
    return isBinaryEqual(rhs);
  }
  /// \brief Less than operator. Compares first multiframe ID, then camera index,
  ///        then keypoint index.
  bool operator<(const KeypointIdentifier & rhs) const
  {

    if (frameId == rhs.frameId) {
      if (cameraIndex == rhs.cameraIndex) {
        return keypointIndex < rhs.keypointIndex;
      } else {
        return cameraIndex < rhs.cameraIndex;
      }
    }
    return frameId < rhs.frameId;
  }

};

struct IsObservedInNFrame {
  IsObservedInNFrame(uint64_t x) : frameId(x) {}
  bool operator()(
      const std::pair<okvis::KeypointIdentifier, uint64_t> &v) const {
    return v.first.frameId == frameId;
  }

 private:
  uint64_t frameId;  ///< Multiframe ID.
};

struct IsObservedInFrame {
  IsObservedInFrame(uint64_t _frameId, size_t _camIdx) :
    frameId(_frameId), cameraIndex(_camIdx) {}
  bool operator()(
      const std::pair<okvis::KeypointIdentifier, uint64_t> &v) const {
    return v.first.frameId == frameId && v.first.cameraIndex == cameraIndex;
  }

 private:
  uint64_t frameId;  ///< Multiframe ID.
  size_t cameraIndex;
};

/// \brief Type to store the result of matching.
struct Match
{
  /**
   * @brief Constructor.
   * @param idxA_ Keypoint index of frame A.
   * @param idxB_ Keypoint index of frame B.
   * @param distance_ Descriptor distance between those two keypoints.
   */
  Match(size_t idxA_, size_t idxB_, float distance_)
      : idxA(idxA_),
        idxB(idxB_),
        distance(distance_)
  {
  }
  size_t idxA;    ///< Keypoint index in frame A.
  size_t idxB;    ///< Keypoint index in frame B.
  float distance; ///< Distance between the keypoints.
};
typedef std::vector<Match> Matches;

/**
 * @brief A type to store information about a point in the world map.
 */
struct MapPoint
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// \brief Default constructor. Point is at origin with quality 0.0 and ID 0.
  MapPoint()
      : id(0),
        quality(0.0),
        distance(0.0),
        anchorStateId(0u),
        anchorCameraId(0u),
        initialized(false)
  {
  }
  /**
   * @brief Constructor.
   * @param id        ID of the point. E.g. landmark ID.
   * @param point     Homogeneous coordinate of the point.
   * @param quality   Quality of the point. Usually between 0 and 1.
   * @param distance  Distance to origin of the frame the coordinates are given in.
   */
  MapPoint(uint64_t id, const Eigen::Vector4d & point,
           double quality, double distance)
      : id(id),
        pointHomog(point),
        quality(quality),
        distance(distance),
        anchorStateId(0u),
        anchorCameraId(0u),
        initialized(false)
  {
  }

  bool trackedInCurrentFrame(uint64_t currentFrameId) const {
    return observations.rbegin()->first.frameId == currentFrameId;
  }

  bool shouldRemove(size_t maxHibernationFrames) const {
    bool toRemove(false);
    if (status.measurementType == swift_vio::FeatureTrackStatus::kMsckfTrack &&
        status.measurementFate == swift_vio::FeatureTrackStatus::kSuccessful) {
      toRemove = true;
    }
    if (status.numMissFrames >= maxHibernationFrames) {
      toRemove = true;
    }
    return toRemove;
  }

  /**
   * @brief shouldChangeAnchor should change the anchor frame of a landmark?
   * @param sortedToRemoveStateIds
   * @return 0 if the landmark is not parameterized with an anchor frame or the
   * anchor frame is NOT one of the state variables to remove, otherwise, the
   * candidate anchor frame id is returned.
   */
  uint64_t
  shouldChangeAnchor(const std::vector<uint64_t> &sortedToRemoveStateIds) const;

  /**
   * @brief goodForMarginalization Is this map point good to be used in the frame marginalization step?
   * This checks if this map point's observations have been used earlier as measurements.
   * @param minCulledFrames minimum frames to be culled in one frame marginalization step.
   * @return true if the map point is good for marginalization.
   */
  bool goodForMarginalization(size_t minCulledFrames) const;

  void updateStatus(uint64_t currentFrameId, size_t minTrackLengthForMsckf,
                    size_t minTrackLengthForSlam);

  void setMeasurementType(swift_vio::FeatureTrackStatus::MeasurementType measurementType) {
    status.measurementType = measurementType;
  }

  void setMeasurementFate(swift_vio::FeatureTrackStatus::MeasurementFate measurementFate) {
    status.measurementFate = measurementFate;
  }

  bool hasObservationInImage(uint64_t frameId, size_t cameraId) const {
    auto iter = observations.lower_bound(okvis::KeypointIdentifier(frameId, cameraId, 0u));
    if (iter != observations.end() && iter->first.frameId == frameId && iter->first.cameraIndex == cameraId) {
      return true;
    }
    return false;
  }

  uint64_t id;            ///< ID of the point. E.g. landmark ID.
  Eigen::Vector4d pointHomog;  ///< Homogeneous coordinate of the point in the World frame.

  double quality;         ///< Quality of the point. Usually between 0 and 1.
  double distance;        ///< Distance to origin of the frame the coordinates are given in.
  std::map<okvis::KeypointIdentifier, uint64_t> observations;   ///< Observations of this point.

  uint64_t anchorStateId;
  size_t anchorCameraId;

  swift_vio::FeatureTrackStatus status;
  bool initialized; // is this landmark initialized in position?
};

typedef std::vector<MapPoint, Eigen::aligned_allocator<MapPoint> > MapPointVector;
typedef std::map<uint64_t, MapPoint, std::less<uint64_t>,
    Eigen::aligned_allocator<MapPoint> > PointMap;
typedef std::map<uint64_t, okvis::kinematics::Transformation, std::less<uint64_t>,
    Eigen::aligned_allocator<okvis::kinematics::Transformation> > TransformationMap;

/// \brief For convenience to pass associations - also contains the 3d points.
struct Observation
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Constructor.
   * @param keypointIdx Keypoint ID.
   * @param keypointMeasurement Image coordinates of keypoint. [pixels]
   * @param keypointSize Keypoint size. Basically standard deviation of the
   *                     image coordinates in pixels.
   * @param cameraIdx Camera index of observed keypoint.
   * @param frameId Frame ID of observed keypoint.
   * @param landmark_W  Associated landmark coordinates in world frame.
   * @param landmarkId  Unique landmark ID
   * @param isInitialized Is the landmark initialized?
   */
  Observation(size_t keypointIdx,
              const Eigen::Vector2d& keypointMeasurement,
              double keypointSize,
              size_t cameraIdx,
              uint64_t frameId,
              const Eigen::Vector4d& landmark_W,
              uint64_t landmarkId, bool isInitialized,
              int num_observations)
      : keypointIdx(keypointIdx),
        cameraIdx(cameraIdx),
        frameId(frameId),
        keypointMeasurement(keypointMeasurement),
        keypointSize(keypointSize),
        landmark_W(landmark_W),
        landmarkId(landmarkId),
        isInitialized(isInitialized),
        numObservations(num_observations)
  {
  }
  Observation()
      : keypointIdx(0),
        cameraIdx(-1),
        frameId(0),
        keypointSize(0),
        landmarkId(0),
        isInitialized(false),
        numObservations(0)
  {
  }
  size_t keypointIdx; ///< Keypoint ID.
  size_t cameraIdx;  ///< index of the camera this point is observed in
  uint64_t frameId;  ///< unique pose block ID == multiframe ID
  Eigen::Vector2d keypointMeasurement;  ///< 2D image keypoint [pixels]
  double keypointSize;  ///< Keypoint size. Basically standard deviation of the image coordinates in pixels.
  Eigen::Vector4d landmark_W;  ///< landmark as homogeneous point in body frame B
  uint64_t landmarkId;  ///< unique landmark ID
  bool isInitialized;   ///< Initialisation status of landmark
  int numObservations; ///< Total number of observations for the landmark.
};
typedef std::vector<Observation, Eigen::aligned_allocator<Observation> > ObservationVector;

// todo: find a better place for this
typedef Eigen::Matrix<double, 9, 1> SpeedAndBiases;
typedef Eigen::Matrix<double, 9, 1> SpeedAndBias;

}  // namespace okvis

#endif /* INCLUDE_OKVIS_FRAMETYPEDEFS_HPP_ */
