/**
 * @file trajopt_default_solver_profile.h
 * @brief
 *
 * @author Levi Armstrong
 * @date December 13, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef TESSERACT_MOTION_PLANNERS_TRAJOPT_DEFAULT_SOLVER_PROFILE_H
#define TESSERACT_MOTION_PLANNERS_TRAJOPT_DEFAULT_SOLVER_PROFILE_H

#include <tesseract_motion_planners/trajopt/profile/trajopt_profile.h>
#include <trajopt_sco/optimizers.hpp>

namespace tesseract_planning
{
/** @brief The contains the default solver parameters available for setting up TrajOpt */
class TrajOptDefaultSolverProfile : public TrajOptSolverProfile
{
public:
  using Ptr = std::shared_ptr<TrajOptDefaultSolverProfile>;
  using ConstPtr = std::shared_ptr<const TrajOptDefaultSolverProfile>;

  /** @brief The Convex solver to use */
  sco::ModelType convex_solver{ sco::ModelType::OSQP };

  /** @brief The convex solver config to use, if nullptr the default settings are used */
  sco::ModelConfig::Ptr convex_solver_config{ nullptr };

  /** @brief Optimization paramters */
  sco::BasicTrustRegionSQPParameters opt_info;

  /** @brief Optimization callbacks */
  std::vector<sco::Optimizer::Callback> callbacks;

  void apply(trajopt::ProblemConstructionInfo& pci) const override;

  tinyxml2::XMLElement* toXML(tinyxml2::XMLDocument& doc) const override;

protected:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive&, const unsigned int);  // NOLINT
};
}  // namespace tesseract_planning

BOOST_CLASS_EXPORT_KEY(tesseract_planning::TrajOptDefaultSolverProfile)

#endif  // TESSERACT_MOTION_PLANNERS_TRAJOPT_DEFAULT_SOLVER_PROFILE_H
