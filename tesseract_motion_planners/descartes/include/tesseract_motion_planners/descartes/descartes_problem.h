/**
 * @file descartes_problem.h
 * @brief
 *
 * @author Levi Armstrong
 * @date June 18, 2020
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
#ifndef TESSERACT_MOTION_PLANNERS_DESCARTES_PROBLEM_H
#define TESSERACT_MOTION_PLANNERS_DESCARTES_PROBLEM_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <descartes_light/core/edge_evaluator.h>
#include <descartes_light/core/state_evaluator.h>
#include <descartes_light/core/waypoint_sampler.h>
#include <thread>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_kinematics/core/fwd.h>
#include <tesseract_environment/fwd.h>

#include <tesseract_scene_graph/scene_state.h>

namespace tesseract_planning
{
template <typename FloatType>
struct DescartesProblem
{
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  // These are required for Tesseract to configure Descartes
  std::shared_ptr<const tesseract_environment::Environment> env;
  tesseract_scene_graph::SceneState env_state;

  // Kinematic Objects
  std::shared_ptr<const tesseract_kinematics::KinematicGroup> manip;

  // These are required for descartes
  std::vector<typename descartes_light::EdgeEvaluator<FloatType>::ConstPtr> edge_evaluators{};
  std::vector<typename descartes_light::WaypointSampler<FloatType>::ConstPtr> samplers{};
  std::vector<typename descartes_light::StateEvaluator<FloatType>::ConstPtr> state_evaluators{};
  int num_threads = static_cast<int>(std::thread::hardware_concurrency());
};
using DescartesProblemF = DescartesProblem<float>;
using DescartesProblemD = DescartesProblem<double>;

}  // namespace tesseract_planning

#endif  // TESSERACT_MOTION_PLANNERS_DESCARTES_PROBLEM_H
