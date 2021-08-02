#include "swift_vio/CameraObservationModel.h"

#include <okvis/ceres/ReprojectionErrorBase.hpp>
#include <swift_vio/ceres/ChordalDistance.hpp>
#include <swift_vio/ceres/EpipolarFactor.hpp>
#include <swift_vio/ceres/ReprojectionErrorWithPap.hpp>
#include <swift_vio/ceres/RsReprojectionError.hpp>
#include <swift_vio/ceres/RSCameraReprojectionError.hpp>

namespace swift_vio {
int CameraObservationModel::toResidualDim(int modelId) {
  switch (modelId) {
#define RESIDUAL_MODEL_CASE(Model)                                             \
  case Model::kModelId:                                                        \
    return Model::kNumResiduals;

    RESIDUAL_MODEL_SWITCH_CASES

#undef RESIDUAL_MODEL_CASE
  }
}

} // namespace swift_vio
