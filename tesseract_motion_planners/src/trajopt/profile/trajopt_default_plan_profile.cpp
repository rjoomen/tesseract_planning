/**
 * @file trajopt_default_plan_profile.cpp
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

#include <tesseract_motion_planners/trajopt/trajopt_utils.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h>

#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/plan_instruction.h>
#include <tesseract_command_language/instruction_type.h>

namespace tesseract_planning
{
void TrajOptDefaultPlanProfile::apply(trajopt::ProblemConstructionInfo& pci,
                                      const Eigen::Isometry3d& cartesian_waypoint,
                                      const Instruction& parent_instruction,
                                      const ManipulatorInfo& manip_info,
                                      const std::vector<std::string>& active_links,
                                      int index)
{
  assert(isPlanInstruction(parent_instruction));
  const auto* base_instruction = parent_instruction.cast_const<PlanInstruction>();
  assert(!(manip_info.isEmpty() && base_instruction->getManipulatorInfo().isEmpty()));
  const ManipulatorInfo& mi =
      (base_instruction->getManipulatorInfo().isEmpty()) ? manip_info : base_instruction->getManipulatorInfo();

  trajopt::TermInfo::Ptr ti{ nullptr };

  /* Check if this cartesian waypoint is dynamic
   * (i.e. defined relative to a frame that will move with the kinematic chain)
   */
  auto it = std::find(active_links.begin(), active_links.end(), mi.working_frame);
  if (it != active_links.end())
  {
    ti = createDynamicCartesianWaypointTermInfo(
        cartesian_waypoint, index, mi.working_frame, mi.tcp, cartesian_coeff, pci.kin->getTipLinkName(), term_type);
  }
  else
  {
    ti = createCartesianWaypointTermInfo(
        cartesian_waypoint, index, mi.working_frame, mi.tcp, cartesian_coeff, pci.kin->getTipLinkName(), term_type);
  }

  if (term_type == trajopt::TermType::TT_CNT)
    pci.cnt_infos.push_back(ti);
  else
    pci.cost_infos.push_back(ti);
}

void TrajOptDefaultPlanProfile::apply(trajopt::ProblemConstructionInfo& pci,
                                      const Eigen::VectorXd& joint_waypoint,
                                      const Instruction& /*parent_instruction*/,
                                      const ManipulatorInfo& /*manip_info*/,
                                      const std::vector<std::string>& /*active_links*/,
                                      int index)
{
  auto ti = createJointWaypointTermInfo(joint_waypoint, index, joint_coeff, term_type);

  if (term_type == trajopt::TermType::TT_CNT)
    pci.cnt_infos.push_back(ti);
  else
    pci.cost_infos.push_back(ti);
}

}  // namespace tesseract_planning