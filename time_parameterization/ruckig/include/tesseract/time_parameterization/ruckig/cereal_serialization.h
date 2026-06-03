#ifndef TESSERACT_TIME_PARAMETERIZATION_RUCKIG_TRAJECTORY_SMOOTHING_CEREAL_SERIALIZATION_H
#define TESSERACT_TIME_PARAMETERIZATION_RUCKIG_TRAJECTORY_SMOOTHING_CEREAL_SERIALIZATION_H

#include <tesseract/time_parameterization/ruckig/ruckig_trajectory_smoothing_profiles.h>

#include <tesseract/common/cereal_serialization.h>

#include <cereal/cereal.hpp>
#include <cereal/types/polymorphic.hpp>

namespace tesseract::time_parameterization
{
template <class Archive>
void serialize(Archive& ar, RuckigTrajectorySmoothingCompositeProfile& obj)
{
  ar(cereal::base_class<tesseract::common::Profile>(&obj));
  ar(cereal::make_nvp("duration_extension_fraction", obj.duration_extension_fraction));
  ar(cereal::make_nvp("max_duration_extension_factor", obj.max_duration_extension_factor));
  ar(cereal::make_nvp("override_limits", obj.override_limits));
  ar(cereal::make_nvp("velocity_limits", obj.velocity_limits));
  ar(cereal::make_nvp("acceleration_limits", obj.acceleration_limits));
  ar(cereal::make_nvp("jerk_limits", obj.jerk_limits));
}
}  // namespace tesseract::time_parameterization

// On Windows and macOS the cereal polymorphic-type registration must be in the header,
// for other platforms registration is in the cpp.
#if defined(_WIN32) || defined(__APPLE__)
#include <tesseract/time_parameterization/ruckig/cereal_serialization_impl.hpp>
#else
CEREAL_FORCE_DYNAMIC_INIT(tesseract_time_parameterization_ruckig_cereal)
#endif

#endif  // TESSERACT_TIME_PARAMETERIZATION_RUCKIG_TRAJECTORY_SMOOTHING_CEREAL_SERIALIZATION_H
