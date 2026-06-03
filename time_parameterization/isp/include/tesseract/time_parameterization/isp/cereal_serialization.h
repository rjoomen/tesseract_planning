#ifndef TESSERACT_TIME_PARAMETERIZATION_ITERATIVE_SPLINE_PARAMETERIZATION_CEREAL_SERIALIZATION_H
#define TESSERACT_TIME_PARAMETERIZATION_ITERATIVE_SPLINE_PARAMETERIZATION_CEREAL_SERIALIZATION_H

#include <tesseract/time_parameterization/isp/iterative_spline_parameterization_profiles.h>

#include <tesseract/common/cereal_serialization.h>

#include <cereal/cereal.hpp>
#include <cereal/types/polymorphic.hpp>

namespace tesseract::time_parameterization
{
template <class Archive>
void serialize(Archive& ar, IterativeSplineParameterizationCompositeProfile& obj)
{
  ar(cereal::base_class<tesseract::common::Profile>(&obj));
  ar(cereal::make_nvp("add_points", obj.add_points));
  ar(cereal::make_nvp("override_limits", obj.override_limits));
  ar(cereal::make_nvp("velocity_limits", obj.velocity_limits));
  ar(cereal::make_nvp("acceleration_limits", obj.acceleration_limits));
  ar(cereal::make_nvp("max_velocity_scaling_factor", obj.max_velocity_scaling_factor));
  ar(cereal::make_nvp("max_acceleration_scaling_factor", obj.max_acceleration_scaling_factor));
  ar(cereal::make_nvp("minimum_time_delta", obj.minimum_time_delta));
}

template <class Archive>
void serialize(Archive& ar, IterativeSplineParameterizationMoveProfile& obj)
{
  ar(cereal::base_class<tesseract::common::Profile>(&obj));
  ar(cereal::make_nvp("max_velocity_scaling_factor", obj.max_velocity_scaling_factor));
  ar(cereal::make_nvp("max_acceleration_scaling_factor", obj.max_acceleration_scaling_factor));
}

}  // namespace tesseract::time_parameterization

// On Windows and macOS the cereal polymorphic-type registration must be in the header,
// for other platforms registration is in the cpp.
#if defined(_WIN32) || defined(__APPLE__)
#include <tesseract/time_parameterization/isp/cereal_serialization_impl.hpp>
#else
CEREAL_FORCE_DYNAMIC_INIT(tesseract_time_parameterization_isp_cereal)
#endif

#endif  // TESSERACT_TIME_PARAMETERIZATION_ITERATIVE_SPLINE_PARAMETERIZATION_CEREAL_SERIALIZATION_H
