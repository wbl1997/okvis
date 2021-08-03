
/**
 * @file implementation/ReprojectionErrorWithPap.hpp
 * @brief Header implementation file for the ReprojectionErrorWithPap class.
 * @author Jianzhu Huai
 */
#include <okvis/ceres/MarginalizationError.hpp>
#include <okvis/kinematics/Transformation.hpp>
#include <okvis/kinematics/operators.hpp>

#include <swift_vio/DirectionFromParallaxAngleJacobian.hpp>
#include <swift_vio/ceres/JacobianHelpers.hpp>
#include <swift_vio/Measurements.hpp>
#include <swift_vio/imu/SimpleImuOdometry.hpp>
#include <swift_vio/imu/SimpleImuPropagationJacobian.hpp>
#include <swift_vio/TransformMultiplyJacobian.hpp>
#include <swift_vio/VectorNormalizationJacobian.hpp>

namespace okvis {
/// \brief ceres Namespace for ceres-related functionality implemented in okvis.
namespace ceres {
template <class GEOMETRY_TYPE, class PROJ_INTRINSIC_MODEL, class EXTRINSIC_MODEL>
ReprojectionErrorWithPap<GEOMETRY_TYPE, PROJ_INTRINSIC_MODEL, EXTRINSIC_MODEL>::ReprojectionErrorWithPap() {}

template <class GEOMETRY_TYPE, class PROJ_INTRINSIC_MODEL, class EXTRINSIC_MODEL>
ReprojectionErrorWithPap<GEOMETRY_TYPE, PROJ_INTRINSIC_MODEL, EXTRINSIC_MODEL>::
    ReprojectionErrorWithPap(
        std::shared_ptr<const camera_geometry_t> cameraGeometry,
        const Eigen::Vector2d& imageObservation,
        const Eigen::Matrix2d& observationCovariance,
        int observationIndex,
        std::shared_ptr<const swift_vio::PointSharedData> pointDataPtr) :
    observationIndex_(observationIndex),
    pointDataPtr_(pointDataPtr) {
  measurement_ = imageObservation;
  covariance_ = observationCovariance;
  Eigen::Matrix2d information = covariance_.inverse();
  // perform the Cholesky decomposition on order to obtain the correct error
  // weighting
  Eigen::LLT<Eigen::Matrix2d> lltOfInformation(information);
  squareRootInformation_ = lltOfInformation.matrixL().transpose();
  cameraGeometryBase_ = cameraGeometry;
}

template <class GEOMETRY_TYPE, class PROJ_INTRINSIC_MODEL, class EXTRINSIC_MODEL>
bool ReprojectionErrorWithPap<GEOMETRY_TYPE, PROJ_INTRINSIC_MODEL, EXTRINSIC_MODEL>::
    Evaluate(double const* const* parameters, double* residuals,
             double** jacobians) const {
  return EvaluateWithMinimalJacobians(parameters, residuals, jacobians, NULL);
}

template <class GEOMETRY_TYPE, class PROJ_INTRINSIC_MODEL, class EXTRINSIC_MODEL>
bool ReprojectionErrorWithPap<GEOMETRY_TYPE, PROJ_INTRINSIC_MODEL, EXTRINSIC_MODEL>::
    EvaluateWithMinimalJacobians(double const* const* parameters,
                                 double* residuals, double** jacobians,
                                 double** jacobiansMinimal) const {
  // We avoid the use of okvis::kinematics::Transformation here due to
  // quaternion normalization and so forth. This only matters in order to be
  // able to check Jacobians with numeric differentiation chained, first w.r.t.
  // q and then d_alpha.

  swift_vio::ParallaxAnglePoint pap;
  pap.set(parameters[3]);
  std::vector<int> anchorObservationIndices =
      pointDataPtr_->anchorObservationIds();
  if (anchorObservationIndices[0] == observationIndex_) {
    Eigen::Vector3d unit_fj = pap.n_.getVec();
    Eigen::Vector2d imagePoint;
    Eigen::Matrix<double, 2, 3> pointJacobian;
    Eigen::Matrix2Xd intrinsicsJacobian;
    okvis::cameras::CameraBase::ProjectionStatus projectStatus =
        cameraGeometryBase_->project(unit_fj, &imagePoint, &pointJacobian,
                                &intrinsicsJacobian);
    bool projectOk = projectStatus ==
                     okvis::cameras::CameraBase::ProjectionStatus::Successful;
    PROJ_INTRINSIC_MODEL::minimalIntrinsicJacobian(&intrinsicsJacobian);
    Eigen::Vector2d error = imagePoint - measurement_;
    // weight
    Eigen::Vector2d weighted_error = squareRootInformation_ * error;
    // assign
    Eigen::Map<Eigen::Vector2d> resvec(residuals);
    resvec = weighted_error;
    bool valid = projectOk;
    if (jacobians != NULL) {
      setJacobiansZero(jacobians, jacobiansMinimal);
      if (!valid) {
        return false;
      }
      // compute de/du, de/dxcam.
      if (jacobians[3]) {
        Eigen::Matrix<double, 2, 3> jMinimal;
        jMinimal.template topLeftCorner<2, 2>() = squareRootInformation_ * pointJacobian * pap.n_.getM();
        jMinimal.col(2).setZero();
        Eigen::Matrix<double, LANDMARK_MODEL::kLocalDim,
                      LANDMARK_MODEL::kGlobalDim, Eigen::RowMajor>
            jLift;
        LANDMARK_MODEL::liftJacobian(parameters[3], jLift.data());
        Eigen::Map<Eigen::Matrix<double, kNumResiduals,
                                 LANDMARK_MODEL::kGlobalDim, Eigen::RowMajor>>
            j(jacobians[3]);
        j = jMinimal * jLift;
        if (jacobiansMinimal) {
          if (jacobiansMinimal[3]) {
            Eigen::Map<
                Eigen::Matrix<double, kNumResiduals, LANDMARK_MODEL::kLocalDim,
                              Eigen::RowMajor>>
                jM(jacobiansMinimal[3]);
            jM = jMinimal;
          }
        }
      }
      if (jacobians[5]) {
        Eigen::Map<ProjectionIntrinsicJacType> j(jacobians[5]);
        j.noalias() =
            squareRootInformation_ *
            intrinsicsJacobian.topLeftCorner<kNumResiduals, kProjectionIntrinsicDim>();
        if (jacobiansMinimal) {
          if (jacobiansMinimal[5]) {
            Eigen::Map<ProjectionIntrinsicJacType> jM(jacobiansMinimal[5]);
            jM = j;
          }
        }
      }
      if (jacobians[6]) {
        Eigen::Map<DistortionJacType> j(jacobians[6]);
        j.noalias() = squareRootInformation_ *
                      intrinsicsJacobian.topRightCorner<kNumResiduals, kDistortionDim>();
        if (jacobiansMinimal) {
          if (jacobiansMinimal[6]) {
            Eigen::Map<DistortionJacType> jM(jacobiansMinimal[6]);
            jM = j;
          }
        }
      }
    }
    return valid;
  }

  Eigen::Matrix<double, 3, 1> t_BC_B(parameters[4][0], parameters[4][1],
                                     parameters[4][2]);
  Eigen::Quaternion<double> q_BC(parameters[4][6], parameters[4][3],
                                 parameters[4][4], parameters[4][5]);
  std::pair<Eigen::Vector3d, Eigen::Quaterniond> pair_T_BC(t_BC_B, q_BC);

  // compute N_{i,j}.
  okvis::kinematics::Transformation T_WBtij =
      pointDataPtr_->T_WBtij(observationIndex_);
  okvis::kinematics::Transformation T_WBtmi =
      pointDataPtr_->T_WBtij(anchorObservationIndices[0]);
  okvis::kinematics::Transformation T_WBtai =
      pointDataPtr_->T_WBtij(anchorObservationIndices[1]);

  swift_vio::TransformMultiplyJacobian T_WCtij_jacobian(
      std::make_pair(T_WBtij.r(), T_WBtij.q()), pair_T_BC);
  swift_vio::TransformMultiplyJacobian T_WCtmi_jacobian(
      std::make_pair(T_WBtmi.r(), T_WBtmi.q()), pair_T_BC);
  swift_vio::TransformMultiplyJacobian T_WCtai_jacobian(
      std::make_pair(T_WBtai.r(), T_WBtai.q()), pair_T_BC);
  std::pair<Eigen::Vector3d, Eigen::Quaterniond> pair_T_WCtij =
      T_WCtij_jacobian.multiply();
  std::pair<Eigen::Vector3d, Eigen::Quaterniond> pair_T_WCtmi =
      T_WCtmi_jacobian.multiply();
  std::pair<Eigen::Vector3d, Eigen::Quaterniond> pair_T_WCtai =
      T_WCtai_jacobian.multiply();

  swift_vio::DirectionFromParallaxAngleJacobian NijFunction(
      pair_T_WCtmi, pair_T_WCtai.first, pair_T_WCtij.first, pap);
  Eigen::Vector3d Nij = NijFunction.evaluate();
  Eigen::Vector3d NijC = pair_T_WCtij.second.conjugate() * Nij;

  Eigen::Vector2d imagePoint;
  Eigen::Matrix<double, 2, 3> pointJacobian;
  Eigen::Matrix2Xd intrinsicsJacobian;
  okvis::cameras::CameraBase::ProjectionStatus projectStatus =
      cameraGeometryBase_->project(NijC, &imagePoint, &pointJacobian,
                              &intrinsicsJacobian);
  bool projectOk = projectStatus ==
                   okvis::cameras::CameraBase::ProjectionStatus::Successful;
  PROJ_INTRINSIC_MODEL::minimalIntrinsicJacobian(&intrinsicsJacobian);
  Eigen::Vector2d error = imagePoint - measurement_;
  // weight
  Eigen::Vector2d weighted_error = squareRootInformation_ * error;
  Eigen::Map<Eigen::Vector2d> resvec(residuals);
  resvec = weighted_error;
  bool valid = projectOk;

  // calculate jacobians, if required
  if (jacobians != NULL) {
    if (!valid) {
      setJacobiansZero(jacobians, jacobiansMinimal);
      return false;
    }
    okvis::kinematics::Transformation T_WBtij_forJac =
        pointDataPtr_->T_WBtij_ForJacobian(observationIndex_);
    okvis::kinematics::Transformation T_WBtmi_forJac =
        pointDataPtr_->T_WBtij_ForJacobian(anchorObservationIndices[0]);
    okvis::kinematics::Transformation T_WBtai_forJac =
        pointDataPtr_->T_WBtij_ForJacobian(anchorObservationIndices[1]);

    swift_vio::TransformMultiplyJacobian T_WCtij_jacobian(
        std::make_pair(T_WBtij_forJac.r(), T_WBtij_forJac.q()), pair_T_BC);
    swift_vio::TransformMultiplyJacobian T_WCtmi_jacobian(
        std::make_pair(T_WBtmi_forJac.r(), T_WBtmi_forJac.q()), pair_T_BC);
    swift_vio::TransformMultiplyJacobian T_WCtai_jacobian(
        std::make_pair(T_WBtai_forJac.r(), T_WBtai_forJac.q()), pair_T_BC);
    std::pair<Eigen::Vector3d, Eigen::Quaterniond> pair_T_WCtij =
        T_WCtij_jacobian.multiply();
    std::pair<Eigen::Vector3d, Eigen::Quaterniond> pair_T_WCtmi =
        T_WCtmi_jacobian.multiply();
    std::pair<Eigen::Vector3d, Eigen::Quaterniond> pair_T_WCtai =
        T_WCtai_jacobian.multiply();
    swift_vio::DirectionFromParallaxAngleJacobian directionFromParallaxAngleJacobian(
        pair_T_WCtmi, pair_T_WCtai.first, pair_T_WCtij.first, pap);
    Eigen::Vector3d Nij_lin = directionFromParallaxAngleJacobian.evaluate();

    Eigen::Matrix3d R_CtijW = pair_T_WCtij.second.toRotationMatrix().transpose();
    Eigen::Matrix<double, 3, 3> dNC_dN = R_CtijW;
    Eigen::Matrix<double, 3, 3> dNC_dtheta_WCtij = R_CtijW * okvis::kinematics::crossMx(Nij_lin);
    Eigen::Matrix<double, kNumResiduals, 3> de_dN = pointJacobian * dNC_dN;

    Eigen::Matrix3d dN_dp_WCtij;
    directionFromParallaxAngleJacobian.dN_dp_WCtij(&dN_dp_WCtij);
    Eigen::Matrix3d dN_dtheta_WCtmi;
    directionFromParallaxAngleJacobian.dN_dtheta_WCmi(&dN_dtheta_WCtmi);
    Eigen::Matrix3d dN_dp_WCtmi;
    directionFromParallaxAngleJacobian.dN_dp_WCmi(&dN_dp_WCtmi);
    Eigen::Matrix3d dN_dp_WCtai;
    directionFromParallaxAngleJacobian.dN_dp_WCai(&dN_dp_WCtai);
    Eigen::Matrix<double, 3, 2> dN_dni;
    directionFromParallaxAngleJacobian.dN_dni(&dN_dni);
    Eigen::Matrix<double, 3, 1> dN_dthetai;
    directionFromParallaxAngleJacobian.dN_dthetai(&dN_dthetai);
    Eigen::Matrix3d dp_WCtij_dp_WBtij = T_WCtij_jacobian.dp_dp_AB();
    Eigen::Matrix3d dp_WCtmi_dp_WBtmi = T_WCtmi_jacobian.dp_dp_AB();
    Eigen::Matrix3d dp_WCtmi_dtheta_WBtmi = T_WCtmi_jacobian.dp_dtheta_AB();
    Eigen::Matrix3d dtheta_WCtmi_dtheta_WBtmi = T_WCtmi_jacobian.dtheta_dtheta_AB();
    Eigen::Matrix3d dp_WCtai_dp_WBtai = T_WCtai_jacobian.dp_dp_AB();
    // T_WBj
    if (jacobians[0]) {
      Eigen::Matrix<double, kNumResiduals, 6> jMinimal;
      Eigen::Matrix3d dtheta_dtheta_WBtij = T_WCtij_jacobian.dtheta_dtheta_AB();
      jMinimal.leftCols<3>() = de_dN * dN_dp_WCtij * dp_WCtij_dp_WBtij;
      if (anchorObservationIndices[1] == observationIndex_) {
        jMinimal.leftCols<3>() += de_dN * dN_dp_WCtai * dp_WCtai_dp_WBtai;
      }
      jMinimal.rightCols<3>() = pointJacobian * dNC_dtheta_WCtij * dtheta_dtheta_WBtij;
      jMinimal = (squareRootInformation_ * jMinimal).eval();
      Eigen::Map<Eigen::Matrix<double, kNumResiduals, 7, Eigen::RowMajor>> j(
          jacobians[0]);
      Eigen::Matrix<double, 6, 7, Eigen::RowMajor> jLift;
      PoseLocalParameterization::liftJacobian(parameters[0], jLift.data());
      j = jMinimal * jLift;
      if (jacobiansMinimal) {
        if (jacobiansMinimal[0]) {
          Eigen::Map<Eigen::Matrix<double, kNumResiduals, 6, Eigen::RowMajor>>
              jM(jacobiansMinimal[0]);
          jM = jMinimal;
        }
      }
    }
    // T_WBm
    if (jacobians[1]) {
      Eigen::Matrix<double, kNumResiduals, 6, Eigen::RowMajor> jMinimal;
      jMinimal.leftCols<3>() =
          squareRootInformation_ * de_dN * dN_dp_WCtmi * dp_WCtmi_dp_WBtmi;
      jMinimal.rightCols<3>() = squareRootInformation_ * de_dN *
                                (dN_dtheta_WCtmi * dtheta_WCtmi_dtheta_WBtmi +
                                 dN_dp_WCtmi * dp_WCtmi_dtheta_WBtmi);
      Eigen::Map<Eigen::Matrix<double, kNumResiduals, 7, Eigen::RowMajor>> j(
          jacobians[1]);
      Eigen::Matrix<double, 6, 7, Eigen::RowMajor> jLift;
      PoseLocalParameterization::liftJacobian(parameters[1], jLift.data());
      j = jMinimal * jLift;
      if (jacobiansMinimal) {
        if (jacobiansMinimal[1]) {
          Eigen::Map<Eigen::Matrix<double, kNumResiduals, 6, Eigen::RowMajor>>
              jM(jacobiansMinimal[1]);
          jM = jMinimal;
        }
      }
    }
    // T_WBa
    if (jacobians[2]) {
      Eigen::Matrix<double, kNumResiduals, 6, Eigen::RowMajor> jMinimal;
      Eigen::Map<Eigen::Matrix<double, kNumResiduals, 7, Eigen::RowMajor>> j(
          jacobians[2]);
      if (anchorObservationIndices[1] == observationIndex_) {
        Eigen::Map<Eigen::Matrix<double, kNumResiduals, 7, Eigen::RowMajor>> jj(
            jacobians[0]);
        j = jj;
      } else {
        jMinimal.leftCols(3) =
            squareRootInformation_ * de_dN * dN_dp_WCtai * dp_WCtai_dp_WBtai;
        jMinimal.rightCols(3).setZero();
        Eigen::Matrix<double, 6, 7, Eigen::RowMajor> jLift;
        PoseLocalParameterization::liftJacobian(parameters[2], jLift.data());
        j = jMinimal * jLift;
      }
      if (jacobiansMinimal) {
        if (jacobiansMinimal[2]) {
          Eigen::Map<Eigen::Matrix<double, kNumResiduals, 6, Eigen::RowMajor>>
              jM(jacobiansMinimal[2]);
          if (anchorObservationIndices[1] == observationIndex_) {
            Eigen::Map<Eigen::Matrix<double, kNumResiduals, 6, Eigen::RowMajor>>
                jjM(jacobiansMinimal[0]);
            jM = jjM;
          } else {
            jM = jMinimal;
          }
        }
      }
    }
    // point landmark
    if (jacobians[3]) {
      Eigen::Map<Eigen::Matrix<double, kNumResiduals,
                               LANDMARK_MODEL::kGlobalDim, Eigen::RowMajor>>
          j(jacobians[3]);
      Eigen::Matrix<double, 3, LANDMARK_MODEL::kLocalDim> dN_dntheta;
      dN_dntheta.template topLeftCorner<3, 2>() = dN_dni;
      dN_dntheta.col(2) = dN_dthetai;
      Eigen::Matrix<double, kNumResiduals, LANDMARK_MODEL::kLocalDim,
                    Eigen::RowMajor> jMinimal = squareRootInformation_ * de_dN * dN_dntheta;
      Eigen::Matrix<double, LANDMARK_MODEL::kLocalDim,
                    LANDMARK_MODEL::kGlobalDim, Eigen::RowMajor>
          jLift;
      LANDMARK_MODEL::liftJacobian(parameters[3], jLift.data());
      j = jMinimal * jLift;
      if (jacobiansMinimal) {
        if (jacobiansMinimal[3]) {
          Eigen::Map<Eigen::Matrix<double, kNumResiduals,
                                   LANDMARK_MODEL::kLocalDim, Eigen::RowMajor>>
              jM(jacobiansMinimal[3]);
          jM = jMinimal;
        }
      }
    }
    // T_BC
    if (jacobians[4]) {
      Eigen::Map<Eigen::Matrix<double, kNumResiduals, 7, Eigen::RowMajor>> j(
          jacobians[4]);
      Eigen::Matrix<double, kNumResiduals, EXTRINSIC_MODEL::kNumParams, Eigen::RowMajor> jMinimal;

      Eigen::Matrix3d dp_WCtmi_dp_BC = T_WCtmi_jacobian.dp_dp_BC();
      Eigen::Matrix3d dp_WCtai_dp_BC = T_WCtai_jacobian.dp_dp_BC();
      Eigen::Matrix3d dp_WCtij_dp_BC = T_WCtij_jacobian.dp_dp_BC();
      jMinimal.template leftCols<3>() =
          squareRootInformation_ * de_dN *
          (dN_dp_WCtmi * dp_WCtmi_dp_BC + dN_dp_WCtai * dp_WCtai_dp_BC +
           dN_dp_WCtij * dp_WCtij_dp_BC);
      Eigen::Matrix<double, EXTRINSIC_MODEL::kNumParams, 7, Eigen::RowMajor> jLift;
      switch (EXTRINSIC_MODEL::kModelId) {
        case swift_vio::Extrinsic_p_CB::kModelId:
          jMinimal.template leftCols<3>() = -jMinimal.template leftCols<3>() *
              pair_T_BC.second.toRotationMatrix();
          EXTRINSIC_MODEL::liftJacobian(parameters[4], jLift.data());
          break;
        case swift_vio::Extrinsic_p_BC_q_BC::kModelId:
        default:
          {

            Eigen::Matrix3d dtheta_WCtmi_dtheta_BC = T_WCtmi_jacobian.dtheta_dtheta_BC();
            Eigen::Matrix3d dp_WCtmi_dtheta_BC = T_WCtmi_jacobian.dp_dtheta_BC();
            Eigen::Matrix3d dp_WCtai_dtheta_BC = T_WCtai_jacobian.dp_dtheta_BC();
            Eigen::Matrix3d dp_WCtij_dtheta_BC = T_WCtij_jacobian.dp_dtheta_BC();
            Eigen::Matrix3d dtheta_WCtij_dtheta_BC = T_WCtij_jacobian.dtheta_dtheta_BC();
            jMinimal.template rightCols<3>() =
                squareRootInformation_ *
                (de_dN * (dN_dtheta_WCtmi * dtheta_WCtmi_dtheta_BC +
                          dN_dp_WCtmi * dp_WCtmi_dtheta_BC +
                          dN_dp_WCtai * dp_WCtai_dtheta_BC +
                          dN_dp_WCtij * dp_WCtij_dtheta_BC) +
                 pointJacobian * dNC_dtheta_WCtij *
                     dtheta_WCtij_dtheta_BC);
            PoseLocalParameterization::liftJacobian(parameters[4], jLift.data());
          }
          break;
      }
      j = jMinimal * jLift;
      if (jacobiansMinimal) {
        if (jacobiansMinimal[4]) {
          Eigen::Map<Eigen::Matrix<double, kNumResiduals, EXTRINSIC_MODEL::kNumParams, Eigen::RowMajor>>
              jM(jacobiansMinimal[4]);
          jM = jMinimal;
        }
      }
    }
    // projection intrinsic
    if (jacobians[5]) {
      Eigen::Map<ProjectionIntrinsicJacType> j(jacobians[5]);
      j = squareRootInformation_ * intrinsicsJacobian.leftCols<kProjectionIntrinsicDim>();
      if (jacobiansMinimal) {
        if (jacobiansMinimal[5]) {
          Eigen::Map<ProjectionIntrinsicJacType> jM(jacobiansMinimal[5]);
          jM = j;
        }
      }
    }
    // distortion
    if (jacobians[6]) {
      Eigen::Map<DistortionJacType> j(jacobians[6]);
      j = squareRootInformation_ * intrinsicsJacobian.rightCols<kDistortionDim>();
      if (jacobiansMinimal) {
        if (jacobiansMinimal[6]) {
          Eigen::Map<DistortionJacType> jM(jacobiansMinimal[6]);
          jM = j;
        }
      }
    }
    // readout time
    Eigen::Vector3d v_WBtij =
        pointDataPtr_->v_WBtij_ForJacobian(observationIndex_);
    Eigen::Vector3d omega_Btij = pointDataPtr_->omega_Btij(observationIndex_);
    Eigen::Vector3d v_WBtmi =
        pointDataPtr_->v_WBtij_ForJacobian(anchorObservationIndices[0]);
    Eigen::Vector3d omega_Btmi =
        pointDataPtr_->omega_Btij(anchorObservationIndices[0]);
    Eigen::Vector3d v_WBtai =
        pointDataPtr_->v_WBtij_ForJacobian(anchorObservationIndices[1]);
    Eigen::Vector3d omega_Btai =
        pointDataPtr_->omega_Btij(anchorObservationIndices[1]);


    T_WCtmi_jacobian.setVelocity(v_WBtmi, omega_Btmi);
    Eigen::Vector3d dp_WCtmi_dt = T_WCtmi_jacobian.dp_dt();
    Eigen::Vector3d dtheta_WCtmi_dt = T_WCtmi_jacobian.dtheta_dt();
    T_WCtij_jacobian.setVelocity(v_WBtij, omega_Btij);
    Eigen::Vector3d dp_WCtij_dt = T_WCtij_jacobian.dp_dt();
    Eigen::Vector3d dtheta_WCtij_dt = T_WCtij_jacobian.dtheta_dt();
    T_WCtai_jacobian.setVelocity(v_WBtai, omega_Btai);
    Eigen::Vector3d dp_WCtai_dt = T_WCtai_jacobian.dp_dt();
    if (jacobians[7]) {
      Eigen::Map<Eigen::Matrix<double, kNumResiduals, 1>> j(jacobians[7]);
      double rowj = pointDataPtr_->normalizedRow(observationIndex_);
      double rowm = pointDataPtr_->normalizedRow(anchorObservationIndices[0]);
      double rowa = pointDataPtr_->normalizedRow(anchorObservationIndices[1]);

      j = squareRootInformation_ * (de_dN *
              (dN_dtheta_WCtmi * dtheta_WCtmi_dt * rowm +
               dN_dp_WCtmi * dp_WCtmi_dt * rowm +
               dN_dp_WCtai * dp_WCtai_dt * rowa +
               dN_dp_WCtij * dp_WCtij_dt * rowj) +
           pointJacobian * dNC_dtheta_WCtij * dtheta_WCtij_dt * rowj);
      if (jacobiansMinimal) {
        if (jacobiansMinimal[7]) {
          Eigen::Map<Eigen::Matrix<double, kNumResiduals, 1>> jM(
              jacobiansMinimal[7]);
          jM = j;
        }
      }
    }
    // camera time delay
    if (jacobians[8]) {
      Eigen::Map<Eigen::Matrix<double, kNumResiduals, 1>> j(jacobians[8]);
      j = squareRootInformation_ * (de_dN *
              (dN_dtheta_WCtmi * dtheta_WCtmi_dt + dN_dp_WCtmi * dp_WCtmi_dt +
               dN_dp_WCtai * dp_WCtai_dt + dN_dp_WCtij * dp_WCtij_dt) +
              pointJacobian * dNC_dtheta_WCtij *
              dtheta_WCtij_dt);
      if (jacobiansMinimal) {
        if (jacobiansMinimal[8]) {
          Eigen::Map<Eigen::Matrix<double, kNumResiduals, 1>> jM(
              jacobiansMinimal[8]);
          jM = j;
        }
      }
    }
    // speed and biases for observing frame.
    if (jacobians[9]) {
      double featureTime =
          pointDataPtr_->normalizedFeatureTime(observationIndex_);
      Eigen::Matrix<double, kNumResiduals, 3> de_dv_WBj =
          de_dN * dN_dp_WCtij * dp_WCtij_dp_WBtij * featureTime;
      if (observationIndex_ == anchorObservationIndices[1]) {
        de_dv_WBj += de_dN * dN_dp_WCtai * dp_WCtai_dp_WBtai * featureTime;
      }
      de_dv_WBj = squareRootInformation_ * de_dv_WBj;
      Eigen::Map<Eigen::Matrix<double, kNumResiduals, 9, Eigen::RowMajor>> j(
          jacobians[9]);
      j.leftCols(3) = de_dv_WBj;
      j.rightCols(6).setZero();
      if (jacobiansMinimal) {
        if (jacobiansMinimal[9]) {
          Eigen::Map<Eigen::Matrix<double, kNumResiduals, 9, Eigen::RowMajor>>
              jM(jacobiansMinimal[9]);
          jM = j;
        }
      }
    }
    // speed and biases for main anchor.
    if (jacobians[10]) {
      double featureTime =
          pointDataPtr_->normalizedFeatureTime(anchorObservationIndices[0]);
      Eigen::Matrix<double, kNumResiduals, 3> de_dv_WBm = squareRootInformation_ * de_dN * dN_dp_WCtmi *
                                  dp_WCtmi_dp_WBtmi * featureTime;
      Eigen::Map<Eigen::Matrix<double, kNumResiduals, 9, Eigen::RowMajor>> j(
          jacobians[10]);
      j.leftCols(3) = de_dv_WBm;
      j.rightCols(6).setZero();
      if (jacobiansMinimal) {
        if (jacobiansMinimal[10]) {
          Eigen::Map<Eigen::Matrix<double, kNumResiduals, 9, Eigen::RowMajor>>
              jM(jacobiansMinimal[10]);
          jM = j;
        }
      }
    }
    // speed and biases for associate anchor.
    if (jacobians[11]) {
      Eigen::Map<Eigen::Matrix<double, kNumResiduals, 9, Eigen::RowMajor>> j(
          jacobians[11]);
      if (observationIndex_ == anchorObservationIndices[1]) {
        Eigen::Map<Eigen::Matrix<double, kNumResiduals, 9, Eigen::RowMajor>> jj(
            jacobians[9]);
        j = jj;
      } else {
        double featureTime =
            pointDataPtr_->normalizedFeatureTime(anchorObservationIndices[1]);
        Eigen::Matrix<double, kNumResiduals, 3> de_dv_WBa = squareRootInformation_ * de_dN *
                                    dN_dp_WCtai * dp_WCtai_dp_WBtai *
                                    featureTime;
        j.leftCols(3) = de_dv_WBa;
        j.rightCols(6).setZero();
      }
      if (jacobiansMinimal) {
        if (jacobiansMinimal[11]) {
          Eigen::Map<Eigen::Matrix<double, kNumResiduals, 9, Eigen::RowMajor>>
              jM(jacobiansMinimal[11]);
          jM = j;
        }
      }
    }
  }
  return valid;
}

template <class GEOMETRY_TYPE, class PROJ_INTRINSIC_MODEL, class EXTRINSIC_MODEL>
void ReprojectionErrorWithPap<GEOMETRY_TYPE, PROJ_INTRINSIC_MODEL, EXTRINSIC_MODEL>::
    setJacobiansZero(double** jacobians, double** jacobiansMinimal) const {
  zeroJacobian<7, 6, kNumResiduals>(0, jacobians, jacobiansMinimal);
  zeroJacobian<7, 6, kNumResiduals>(1, jacobians, jacobiansMinimal);
  zeroJacobian<7, 6, kNumResiduals>(2, jacobians, jacobiansMinimal);
  zeroJacobian<LANDMARK_MODEL::kGlobalDim, LANDMARK_MODEL::kLocalDim, kNumResiduals>(3, jacobians, jacobiansMinimal);
  zeroJacobian<7, EXTRINSIC_MODEL::kNumParams, kNumResiduals>(4, jacobians, jacobiansMinimal);
  zeroJacobian<PROJ_INTRINSIC_MODEL::kNumParams,
               PROJ_INTRINSIC_MODEL::kNumParams, kNumResiduals>(5, jacobians, jacobiansMinimal);
  zeroJacobian<kDistortionDim, kDistortionDim, kNumResiduals>(6, jacobians, jacobiansMinimal);
  zeroJacobian<1, 1, kNumResiduals>(7, jacobians, jacobiansMinimal);
  zeroJacobian<1, 1, kNumResiduals>(8, jacobians, jacobiansMinimal);
  zeroJacobian<9, 9, kNumResiduals>(9, jacobians, jacobiansMinimal);
  zeroJacobian<9, 9, kNumResiduals>(10, jacobians, jacobiansMinimal);
  zeroJacobian<9, 9, kNumResiduals>(11, jacobians, jacobiansMinimal);
}
}  // namespace ceres
}  // namespace okvis
