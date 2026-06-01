#ifndef TESSERACT_TIME_PARAMETERIZATION_CONSTANT_TCP_SPEED_PARAMETERIZATION_CEREAL_SERIALIZATION_H
#define TESSERACT_TIME_PARAMETERIZATION_CONSTANT_TCP_SPEED_PARAMETERIZATION_CEREAL_SERIALIZATION_H

#include <tesseract/time_parameterization/kdl/constant_tcp_speed_parameterization_profiles.h>

#include <tesseract/common/cereal_serialization.h>

#include <cereal/cereal.hpp>
#include <cereal/types/polymorphic.hpp>

namespace tesseract::time_parameterization
{
template <class Archive>
void serialize(Archive& ar, ConstantTCPSpeedParameterizationCompositeProfile& obj)
{
  ar(cereal::base_class<tesseract::common::Profile>(&obj));
  ar(cereal::make_nvp("max_translational_velocity", obj.max_translational_velocity));
  ar(cereal::make_nvp("max_rotational_velocity", obj.max_rotational_velocity));
  ar(cereal::make_nvp("max_translational_acceleration", obj.max_translational_acceleration));
  ar(cereal::make_nvp("max_rotational_acceleration", obj.max_rotational_acceleration));
  ar(cereal::make_nvp("max_velocity_scaling_factor", obj.max_velocity_scaling_factor));
  ar(cereal::make_nvp("max_acceleration_scaling_factor", obj.max_acceleration_scaling_factor));
}

}  // namespace tesseract::time_parameterization

// On Windows the cereal polymorphic-type registration must be in the header,
// for other platforms registration is in the cpp.
#ifdef _WIN32
#include <tesseract/time_parameterization/kdl/cereal_serialization_impl.hpp>
#else
CEREAL_FORCE_DYNAMIC_INIT(tesseract_time_parameterization_kdl_cereal)
#endif

#endif  // TESSERACT_TIME_PARAMETERIZATION_CONSTANT_TCP_SPEED_PARAMETERIZATION_CEREAL_SERIALIZATION_H
