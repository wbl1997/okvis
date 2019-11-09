
/**
 * @file ceres/EpipolarFactor.hpp
 * @brief Header file for the EpipolarFactor class.
 * @author Jianzhu Huai
 */

#ifndef INCLUDE_OKVIS_CERES_EPIPOLAR_FACTOR_HPP_
#define INCLUDE_OKVIS_CERES_EPIPOLAR_FACTOR_HPP_

#include <vector>
#include <memory>

#include <Eigen/Core>
#include <Eigen/StdVector>

#include <ceres/ceres.h>
#include <okvis/Measurements.hpp>

#include <okvis/assert_macros.hpp>
#include <okvis/ceres/PoseLocalParameterization.hpp>
#include <okvis/ceres/ErrorInterface.hpp>

namespace okvis {
namespace ceres {

/**
 * \brief The 1D epipolar error.
 * \tparam GEOMETRY_TYPE The camera gemetry type.
 * The constant params are passed into the residual error through the constructor interface.
 * The variable params are reflected in terms of dim in the SizedCostFunction base class.
 * The Jacobians are computed according to these dims except for the reparameterized pose.
 * The variable params will be passed to the evaluate function as scalar
 * pointers so they can be stored as vector<scalar> or Eigen::Vector.
 */
template <class GEOMETRY_TYPE, class PROJ_INTRINSIC_MODEL, class EXTRINSIC_MODEL>
class EpipolarFactor
    : public ::ceres::SizedCostFunction<
          1 /* number of residuals */, 7 /* left pose */, 7 /* right pose */,
          EXTRINSIC_MODEL::kGlobalDim /* variable dim of extrinsics */,
          PROJ_INTRINSIC_MODEL::kNumParams /* variable dim of proj intrinsics
                                              (e.g., f, cx, cy) */,
          GEOMETRY_TYPE::distortion_t::NumDistortionIntrinsics,
          1 /* frame readout time */,
          1 /* time offset between visual and inertial data */,
          9 /* velocity and biases for left pose */,
          9 /* velocity and biases for right pose */
          >,
      public ErrorInterface {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  OKVIS_DEFINE_EXCEPTION(Exception,std::runtime_error)

  /// \brief Make the camera geometry type accessible.
  typedef GEOMETRY_TYPE camera_geometry_t;

  static const int kDistortionDim = GEOMETRY_TYPE::distortion_t::NumDistortionIntrinsics;

  /// \brief The base class type.
  typedef ::ceres::SizedCostFunction<
      1, 7, 7, EXTRINSIC_MODEL::kGlobalDim, PROJ_INTRINSIC_MODEL::kNumParams,
      kDistortionDim, 1, 1, 9, 9>
      base_t;

  /// \brief Number of residuals (2)
  static const int kNumResiduals = 1;

  /// \brief The keypoint type (measurement type).
  typedef Eigen::Vector2d keypoint_t;

  /// \brief Measurement type (1D).
  typedef double measurement_t;

  /// \brief Covariance / information matrix type.
  typedef double covariance_t;

  /// \brief Default constructor.
  EpipolarFactor();

  /**
   * @brief EpipolarFactor Construct with measurement and information matrix
   * @param cameraGeometry The underlying camera geometry. Its copy as a member
   *     of this class will be updated each call to Evaluate()
   * @param measurement12 left and right 2d measurements
   * @param covariance12 left and right 2d covariance for 2d meas
   * @param imuMeasCanopy imu measurements in neighborhoods of the left and
   *     right stateEpochs
   * @param T_SC_base reference extrinsic parameters, needed because
   *     EXTRINSIC_MODEL may be a subset of T_SC
   * @param stateEpoch left and right state timestamps
   * @param tdAtCreation left and right reference td
   * @param gravityMag magnitude of gravity
   */
  EpipolarFactor(
      std::shared_ptr<const camera_geometry_t> cameraGeometry,
      const std::vector<Eigen::Vector2d,
                        Eigen::aligned_allocator<Eigen::Vector2d>>&
          measurement12,
      const std::vector<Eigen::Matrix2d,
                        Eigen::aligned_allocator<Eigen::Matrix2d>>&
          covariance12,
      std::vector<std::shared_ptr<const okvis::ImuMeasurementDeque>> imuMeasCanopy,
      const okvis::kinematics::Transformation& T_SC_base,
      const std::vector<okvis::Time>& stateEpoch,
      const std::vector<double>& tdAtCreation, double gravityMag);

  /// \brief Trivial destructor.
  virtual ~EpipolarFactor()
  {
  }


  /// \brief Set the underlying camera model.
  /// @param[in] cameraGeometry The camera geometry.
  void setCameraGeometry(
      std::shared_ptr<const camera_geometry_t> cameraGeometry)
  {
    cameraGeometryBase_ = std::make_shared<camera_geometry_t>(*cameraGeometry);
  }

  // getters
  /// \brief Get the information matrix.
  /// \return The information (weight) matrix.
  virtual const covariance_t& information() const
  {
    return information_;
  }

  // error term and Jacobian implementation
  /**
   * @brief This evaluates the error term and additionally computes the Jacobians.
   * @param parameters Pointer to the parameters (see ceres)
   * @param residuals Pointer to the residual vector (see ceres)
   * @param jacobians Pointer to the Jacobians (see ceres)
   * @return success of th evaluation.
   */
  virtual bool Evaluate(double const* const * parameters, double* residuals,
                        double** jacobians) const;

  /**
   * @brief This evaluates the error term and additionally computes
   *        the Jacobians in the minimal internal representation.
   * @param parameters Pointer to the parameters (see ceres)
   * @param residuals Pointer to the residual vector (see ceres)
   * @param jacobians Pointer to the Jacobians (see ceres)
   * @param jacobiansMinimal Pointer to the minimal Jacobians (equivalent to jacobians).
   * @return Success of the evaluation.
   */
  virtual bool EvaluateWithMinimalJacobians(double const* const * parameters,
                                            double* residuals,
                                            double** jacobians,
                                            double** jacobiansMinimal) const;

  void setJacobiansZero(double** jacobians, double** jacobiansMinimal) const;

  // sizes
  /// \brief Residual dimension.
  size_t residualDim() const
  {
    return kNumResiduals;
  }

  /// \brief Number of parameter blocks.
  size_t parameterBlocks() const
  {
    return base_t::parameter_block_sizes().size();
  }

  /// \brief Dimension of an individual parameter block.
  /// @param[in] parameterBlockId ID of the parameter block of interest.
  /// \return The dimension.
  size_t parameterBlockDim(size_t parameterBlockId) const
  {
    return base_t::parameter_block_sizes().at(parameterBlockId);
  }

  /// @brief Residual block type as string
  virtual std::string typeInfo() const
  {
    return "EpipolarFactor";
  }

 protected:
  std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> measurement_;
  std::vector<Eigen::Matrix2d, Eigen::aligned_allocator<Eigen::Matrix2d>> covariance_;

  // An independent copy of the camera model from other EpipolarFactor's copies.
  // The camera model is volatile and updated in every Evaluate() step.
  mutable std::shared_ptr<camera_geometry_t> cameraGeometryBase_;
  // T_SC_base_ is volatile and updated in every Evaluate() step.
  // assume the two measurements are made by the same camera
  mutable okvis::kinematics::Transformation T_SC_base_;
  // const after initialization
  std::vector<std::shared_ptr<const okvis::ImuMeasurementDeque>> imuMeasCanopy_;

  // weighting related, they will be computed along with the residual
  double information_; ///< The information matrix.
  double squareRootInformation_; ///< The square root information matrix.

  std::vector<okvis::Time> stateEpoch_; ///< The timestamp of the set of robot states related to this error term.
  std::vector<double> tdAtCreation_;

  const double gravityMag_; ///< gravity in the world frame is [0, 0, -gravityMag_].

  /**
   * @brief computePoseAtExposure compute T_WS at the time of exposure
   * @param[in/out] pairT_WS in order to avoid use of okvis::Transformation
   * @param[in] parameters as used in Evaluate()
   * @param[in] index 0 for left camera, 1 for right
   */
  void computePoseAtExposure(std::pair<Eigen::Quaternion<double>,
                                       Eigen::Matrix<double, 3, 1>>* pairT_WS,
                             double const* const* parameters, int index) const;
};

}  // namespace ceres
}  // namespace okvis

#include "implementation/EpipolarFactor.hpp"
#endif /* INCLUDE_OKVIS_CERES_EPIPOLAR_FACTOR_HPP_ */