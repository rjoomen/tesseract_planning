/*
 * Test to verify endpoint velocity/acceleration behavior in Tesseract TOTG
 *
 * This test checks whether Tesseract has the same behavior as MoveIt2 Jazzy
 * where first and last waypoints may have non-zero velocities/accelerations.
 *
 * Reference: https://github.com/moveit/moveit2/issues/3014
 */

#include <iostream>
#include <iomanip>
#include <tesseract_time_parameterization/totg/time_optimal_trajectory_generation.h>
#include <tesseract_time_parameterization/totg/time_optimal_trajectory_generation_profiles.h>
#include <tesseract_common/resource_locator.h>
#include <tesseract_common/profile_dictionary.h>
#include <tesseract_environment/environment.h>
#include <tesseract_command_language/poly/state_waypoint_poly.h>
#include <tesseract_command_language/poly/move_instruction_poly.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_time_parameterization/core/instructions_trajectory.h>

using namespace tesseract_planning;

void printEndpointInfo(const std::string& label,
                       const CompositeInstruction& program,
                       int index = -1)
{
  if (index == -1)
    index = static_cast<int>(program.size()) - 1;

  auto instruction = program[index].as<MoveInstructionPoly>();
  auto waypoint = instruction.getWaypoint().as<StateWaypointPoly>();

  std::cout << "\n" << label << " (index " << index << "):\n";
  std::cout << "  Time: " << std::setprecision(10) << waypoint.getTime() << " s\n";
  std::cout << "  Position: " << waypoint.getPosition().transpose() << "\n";
  std::cout << "  Velocity: " << waypoint.getVelocity().transpose() << "\n";
  std::cout << "  Acceleration: " << waypoint.getAcceleration().transpose() << "\n";

  // Check if velocity/acceleration are zero
  bool vel_zero = waypoint.getVelocity().isZero(1e-9);
  bool acc_zero = waypoint.getAcceleration().isZero(1e-9);

  std::cout << "  Velocity is zero: " << (vel_zero ? "YES" : "NO") << "\n";
  std::cout << "  Acceleration is zero: " << (acc_zero ? "YES" : "NO") << "\n";

  if (!vel_zero || !acc_zero)
  {
    std::cout << "  ⚠️  WARNING: Non-zero endpoint detected!\n";
  }
}

CompositeInstruction createTestTrajectory(const std::vector<Eigen::VectorXd>& waypoints,
                                          const tesseract_common::ManipulatorInfo& manip_info)
{
  std::vector<std::string> joint_names = { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6" };

  CompositeInstruction program;
  program.setManipulatorInfo(manip_info);

  for (const auto& wp : waypoints)
  {
    StateWaypoint swp(joint_names, wp);
    program.push_back(MoveInstruction(swp, MoveInstructionType::FREESPACE));
  }

  return program;
}

int main(int argc, char** argv)
{
  std::cout << "========================================\n";
  std::cout << "Tesseract TOTG Endpoint Behavior Test\n";
  std::cout << "========================================\n\n";

  // Setup environment
  auto locator = std::make_shared<tesseract_common::GeneralResourceLocator>();
  auto env = std::make_shared<tesseract_environment::Environment>();

  std::filesystem::path urdf_path(
      locator->locateResource("package://tesseract_support/urdf/abb_irb2400.urdf")->getFilePath());
  std::filesystem::path srdf_path(
      locator->locateResource("package://tesseract_support/urdf/abb_irb2400.srdf")->getFilePath());

  if (!env->init(urdf_path, srdf_path, locator))
  {
    std::cerr << "Failed to initialize environment\n";
    return 1;
  }

  tesseract_common::ManipulatorInfo manip;
  manip.manipulator = "manipulator";
  manip.working_frame = "base_link";
  manip.tcp_frame = "tool0";

  // Setup profile
  auto profile = std::make_shared<TimeOptimalTrajectoryGenerationCompositeProfile>();
  profile->path_tolerance = 0.1;
  profile->min_angle_change = 1e-3;
  profile->override_limits = true;
  profile->velocity_limits = Eigen::MatrixX2d(6, 2);
  profile->velocity_limits.col(0) = -1 * Eigen::VectorXd::Ones(6);
  profile->velocity_limits.col(1) = Eigen::VectorXd::Ones(6);
  profile->acceleration_limits = Eigen::MatrixX2d(6, 2);
  profile->acceleration_limits.col(0) = -1 * Eigen::VectorXd::Ones(6);
  profile->acceleration_limits.col(1) = Eigen::VectorXd::Ones(6);

  tesseract_common::ProfileDictionary profiles;
  profiles.addProfile("TOTG", DEFAULT_PROFILE_KEY, profile);

  TimeOptimalTrajectoryGeneration solver("TOTG");

  // ========================================
  // Test 1: Simple A -> B trajectory
  // ========================================
  std::cout << "TEST 1: Simple A -> B Trajectory\n";
  std::cout << "-----------------------------------\n";

  std::vector<Eigen::VectorXd> waypoints1;
  Eigen::VectorXd wp(6);

  wp << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  waypoints1.push_back(wp);

  wp << 1.0, 0.5, -0.5, 0.2, -0.3, 0.1;
  waypoints1.push_back(wp);

  CompositeInstruction traj1 = createTestTrajectory(waypoints1, manip);

  if (!solver.compute(traj1, *env, profiles))
  {
    std::cerr << "ERROR: Failed to compute trajectory 1\n";
    return 1;
  }

  printEndpointInfo("First waypoint", traj1, 0);
  printEndpointInfo("Last waypoint", traj1);

  // ========================================
  // Test 2: A -> B -> C trajectory
  // ========================================
  std::cout << "\n\nTEST 2: A -> B -> C Trajectory\n";
  std::cout << "-----------------------------------\n";

  std::vector<Eigen::VectorXd> waypoints2;

  wp << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  waypoints2.push_back(wp);

  wp << 1.0, 0.5, -0.5, 0.2, -0.3, 0.1;
  waypoints2.push_back(wp);

  wp << 0.5, 1.0, -1.0, 0.4, -0.6, 0.2;
  waypoints2.push_back(wp);

  CompositeInstruction traj2 = createTestTrajectory(waypoints2, manip);

  if (!solver.compute(traj2, *env, profiles))
  {
    std::cerr << "ERROR: Failed to compute trajectory 2\n";
    return 1;
  }

  printEndpointInfo("First waypoint", traj2, 0);
  printEndpointInfo("Last waypoint", traj2);

  // ========================================
  // Test 3: A -> B -> A (return to start)
  // ========================================
  std::cout << "\n\nTEST 3: A -> B -> A (Return to Start)\n";
  std::cout << "-----------------------------------\n";

  std::vector<Eigen::VectorXd> waypoints3;

  wp << 0.0, 0.7, -2.1, 0.0, -0.25, 0.0;
  waypoints3.push_back(wp);

  wp << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  waypoints3.push_back(wp);

  wp << 0.0, 0.70001, -2.1, 0.0, -0.25, 0.0;
  waypoints3.push_back(wp);

  CompositeInstruction traj3 = createTestTrajectory(waypoints3, manip);

  if (!solver.compute(traj3, *env, profiles))
  {
    std::cerr << "ERROR: Failed to compute trajectory 3\n";
    return 1;
  }

  printEndpointInfo("First waypoint", traj3, 0);
  printEndpointInfo("Middle waypoint", traj3, 1);
  printEndpointInfo("Last waypoint", traj3);

  // ========================================
  // Test 4: Concatenation scenario
  // ========================================
  std::cout << "\n\nTEST 4: Concatenation Scenario\n";
  std::cout << "-----------------------------------\n";
  std::cout << "Testing two separate trajectories that share an endpoint:\n";

  // Trajectory 4a: Start -> Mid
  std::vector<Eigen::VectorXd> waypoints4a;
  wp << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  waypoints4a.push_back(wp);
  wp << 1.0, 1.0, 1.0, 0.0, 0.0, 0.0;
  waypoints4a.push_back(wp);

  CompositeInstruction traj4a = createTestTrajectory(waypoints4a, manip);
  if (!solver.compute(traj4a, *env, profiles))
  {
    std::cerr << "ERROR: Failed to compute trajectory 4a\n";
    return 1;
  }

  std::cout << "\nTrajectory A (start to mid):\n";
  printEndpointInfo("  Start", traj4a, 0);
  printEndpointInfo("  End", traj4a);

  // Trajectory 4b: Mid -> End (starts where 4a ended)
  std::vector<Eigen::VectorXd> waypoints4b;
  wp << 1.0, 1.0, 1.0, 0.0, 0.0, 0.0;  // Same as end of 4a
  waypoints4b.push_back(wp);
  wp << 2.0, 0.5, 0.5, 0.0, 0.0, 0.0;
  waypoints4b.push_back(wp);

  CompositeInstruction traj4b = createTestTrajectory(waypoints4b, manip);
  if (!solver.compute(traj4b, *env, profiles))
  {
    std::cerr << "ERROR: Failed to compute trajectory 4b\n";
    return 1;
  }

  std::cout << "\nTrajectory B (mid to end):\n";
  printEndpointInfo("  Start", traj4b, 0);
  printEndpointInfo("  End", traj4b);

  // Check for discontinuity
  auto traj4a_end = traj4a.back().as<MoveInstructionPoly>().getWaypoint().as<StateWaypointPoly>();
  auto traj4b_start = traj4b.front().as<MoveInstructionPoly>().getWaypoint().as<StateWaypointPoly>();

  std::cout << "\n  Discontinuity Check:\n";
  std::cout << "    Position difference: "
            << (traj4a_end.getPosition() - traj4b_start.getPosition()).norm() << "\n";
  std::cout << "    Velocity at A end: " << traj4a_end.getVelocity().norm() << "\n";
  std::cout << "    Velocity at B start: " << traj4b_start.getVelocity().norm() << "\n";
  std::cout << "    Acceleration at A end: " << traj4a_end.getAcceleration().norm() << "\n";
  std::cout << "    Acceleration at B start: " << traj4b_start.getAcceleration().norm() << "\n";

  if (traj4a_end.getVelocity().norm() > 1e-6 || traj4b_start.getVelocity().norm() > 1e-6)
  {
    std::cout << "\n  ⚠️  WARNING: Velocity discontinuity detected!\n";
    std::cout << "  This would cause issues when concatenating trajectories.\n";
    std::cout << "  Similar to MoveIt2 Jazzy behavior (Issue #3014)\n";
  }
  else
  {
    std::cout << "\n  ✓ Both endpoints have zero velocity - safe for concatenation\n";
  }

  // ========================================
  // Summary
  // ========================================
  std::cout << "\n\n========================================\n";
  std::cout << "SUMMARY\n";
  std::cout << "========================================\n";
  std::cout << "Tesseract TOTG endpoint behavior:\n";

  auto test1_start = traj1.front().as<MoveInstructionPoly>().getWaypoint().as<StateWaypointPoly>();
  auto test1_end = traj1.back().as<MoveInstructionPoly>().getWaypoint().as<StateWaypointPoly>();

  bool start_zero_vel = test1_start.getVelocity().isZero(1e-9);
  bool end_zero_vel = test1_end.getVelocity().isZero(1e-9);

  std::cout << "  - Start points have zero velocity: " << (start_zero_vel ? "YES" : "NO") << "\n";
  std::cout << "  - End points have zero velocity: " << (end_zero_vel ? "YES" : "NO") << "\n";

  if (start_zero_vel && end_zero_vel)
  {
    std::cout << "\n  ✓ Tesseract behaves like MoveIt2 Humble (zero endpoints)\n";
    std::cout << "  ✓ Safe for trajectory concatenation\n";
  }
  else
  {
    std::cout << "\n  ⚠️  Tesseract behaves like MoveIt2 Jazzy (non-zero endpoints)\n";
    std::cout << "  ⚠️  May cause issues when concatenating trajectories\n";
    std::cout << "  ⚠️  Manual endpoint zeroing required for concatenation\n";
  }

  std::cout << "\n========================================\n\n";

  return 0;
}
