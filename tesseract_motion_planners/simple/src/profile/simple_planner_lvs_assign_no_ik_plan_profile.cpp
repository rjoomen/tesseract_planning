/**
 * @file simple_planner_lvs_assign_no_ik_plan_profile.cpp
 * @brief
 *
 * @author Roelof Oomen
 * @date May 29, 2024
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2024, ROS Industrial Consortium
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

#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/poly/move_instruction_poly.h>
#include <tesseract_kinematics/core/kinematic_group.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_lvs_assign_no_ik_plan_profile.h>
#include <tesseract_motion_planners/simple/interpolation.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_motion_planners/core/types.h>

namespace tesseract_planning
{
SimplePlannerLVSAssignNoIKPlanProfile::SimplePlannerLVSAssignNoIKPlanProfile(
    double state_longest_valid_segment_length,
    double translation_longest_valid_segment_length,
    double rotation_longest_valid_segment_length,
    int min_steps,
    int max_steps)
  : state_longest_valid_segment_length(state_longest_valid_segment_length)
  , translation_longest_valid_segment_length(translation_longest_valid_segment_length)
  , rotation_longest_valid_segment_length(rotation_longest_valid_segment_length)
  , min_steps(min_steps)
  , max_steps(max_steps)
{
}

std::vector<MoveInstructionPoly>
SimplePlannerLVSAssignNoIKPlanProfile::generate(const MoveInstructionPoly& prev_instruction,
                                                const MoveInstructionPoly& /*prev_seed*/,
                                                const MoveInstructionPoly& base_instruction,
                                                const InstructionPoly& /*next_instruction*/,
                                                const PlannerRequest& request,
                                                const tesseract_common::ManipulatorInfo& global_manip_info) const
{
  JointGroupInstructionInfo prev(prev_instruction, request, global_manip_info);
  JointGroupInstructionInfo base(base_instruction, request, global_manip_info);

  Eigen::VectorXd j1;
  Eigen::Isometry3d p1_world;
  bool has_j1 = false;
  if (prev.has_cartesian_waypoint)
  {
    p1_world = prev.extractCartesianPose();
    if (prev.instruction.getWaypoint().as<CartesianWaypointPoly>().hasSeed())
    {
      j1 = prev.instruction.getWaypoint().as<CartesianWaypointPoly>().getSeed().position;
      has_j1 = true;
    }
  }
  else
  {
    j1 = prev.extractJointPosition();
    p1_world = prev.calcCartesianPose(j1);
    has_j1 = true;
  }

  Eigen::VectorXd j2;
  Eigen::Isometry3d p2_world;
  bool has_j2 = false;
  if (base.has_cartesian_waypoint)
  {
    p2_world = base.extractCartesianPose();
    if (base.instruction.getWaypoint().as<CartesianWaypointPoly>().hasSeed())
    {
      j2 = base.instruction.getWaypoint().as<CartesianWaypointPoly>().getSeed().position;
      has_j2 = true;
    }
  }
  else
  {
    j2 = base.extractJointPosition();
    p2_world = base.calcCartesianPose(j2);
    has_j2 = true;
  }

  double trans_dist = (p2_world.translation() - p1_world.translation()).norm();
  double rot_dist = Eigen::Quaterniond(p1_world.linear()).angularDistance(Eigen::Quaterniond(p2_world.linear()));
  int trans_steps = int(trans_dist / translation_longest_valid_segment_length) + 1;
  int rot_steps = int(rot_dist / rotation_longest_valid_segment_length) + 1;
  int steps = std::max(trans_steps, rot_steps);

  if (has_j1 && has_j2)
  {
    double joint_dist = (j2 - j1).norm();
    int joint_steps = int(joint_dist / state_longest_valid_segment_length) + 1;
    steps = std::max(steps, joint_steps);
  }

  Eigen::MatrixXd states;
  if (has_j2)
  {
    states = j2.replicate(1, steps + 1);
  }
  else if (has_j1)
  {
    states = j1.replicate(1, steps + 1);
  }
  else
  {
    Eigen::VectorXd seed = request.env_state.getJointValues(base.manip->getJointNames());
    tesseract_common::enforceLimits<double>(seed, base.manip->getLimits().joint_limits);
    states = seed.replicate(1, steps + 1);
  }

  // Linearly interpolate in cartesian space if linear move
  if (base_instruction.isLinear())
  {
    tesseract_common::VectorIsometry3d poses = interpolate(p1_world, p2_world, steps);
    for (auto& pose : poses)
      pose = base.working_frame_transform.inverse() * pose;

    assert(poses.size() == states.cols());
    return getInterpolatedInstructions(poses, base.manip->getJointNames(), states, base.instruction);
  }

  return getInterpolatedInstructions(base.manip->getJointNames(), states, base.instruction);
}

}  // namespace tesseract_planning
