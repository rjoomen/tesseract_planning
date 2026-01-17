# Ruckig Implementation Comparison: Tesseract vs MoveIt2

## Executive Summary

The Tesseract Ruckig implementation is based on MoveIt2's code but is missing several critical bug fixes and improvements that were added to MoveIt2 between 2022-2023. This document outlines the key differences and missing features.

## Missing Features and Bug Fixes in Tesseract

### 1. **Overshoot Mitigation** (Added to MoveIt2: March 27, 2023)

**Status:** Missing in Tesseract

**What it does:**
MoveIt2 includes an optional overshoot detection and mitigation system that checks if the smoothed trajectory overshoots the target waypoint positions.

**MoveIt2 Implementation:**
- `checkOvershoot()` function that samples the trajectory at `OVERSHOOT_CHECK_PERIOD` (0.01s) intervals
- Checks if trajectory crosses target position and exceeds `overshoot_threshold` (default 0.01 radians)
- If overshoot detected, extends trajectory duration similar to other failure cases
- Controlled by `mitigate_overshoot` parameter (default: false)

**Location in MoveIt2:**
- `ruckig_traj_smoothing.cpp:482-502` - checkOvershoot() function
- `ruckig_traj_smoothing.cpp:256-261` - overshoot check in main loop

**Impact:**
Without this, Tesseract trajectories may overshoot waypoints, causing the robot to briefly move past target positions before settling, which can be problematic for precision tasks.

---

### 2. **Termination Condition Bug** (Fixed in MoveIt2: March 10, 2023)

**Status:** Partially affected in Tesseract

**The Bug:**
The original code only checked for `Result::Finished` as a successful termination condition. However, Ruckig can return `Result::Working` for valid trajectory segments that require more than one timestep.

**MoveIt2 Fix:**
```cpp
// Both Working and Finished are acceptable
if ((waypoint_idx == num_waypoints - 2) &&
    (ruckig_result == ruckig::Result::Working || ruckig_result == ruckig::Result::Finished))
{
    smoothing_complete = true;
}
```

**Tesseract Current Code:**
```cpp
// Only checks for Finished
if ((waypoint_idx == static_cast<Eigen::Index>(num_waypoints) - 2) &&
    ruckig_result == ruckig::Result::Finished)
{
    smoothing_complete = true;
}
```

**Impact:**
Tesseract may unnecessarily extend trajectory duration even when Ruckig successfully computed a valid path that returns `Result::Working`.

---

### 3. **Duration Extension Optimization** (Added to MoveIt2: March 16, 2023)

**Status:** Missing in Tesseract

**The Problem:**
Original implementation extended the duration of ALL remaining waypoints when Ruckig failed to reach a single waypoint.

**MoveIt2 Optimization:**
```cpp
void extendTrajectoryDuration(const double duration_extension_factor, size_t waypoint_idx,
                              const size_t num_dof, const std::vector<int>& move_group_idx,
                              const robot_trajectory::RobotTrajectory& original_trajectory,
                              robot_trajectory::RobotTrajectory& trajectory)
{
    // Only extends the specific failed segment at waypoint_idx + 1
    trajectory.setWayPointDurationFromPrevious(waypoint_idx + 1,
                                               duration_extension_factor *
                                               original_trajectory.getWayPointDurationFromPrevious(waypoint_idx + 1));
    // Re-calculates velocity and acceleration for just that waypoint
}
```

**Tesseract Current Approach:**
```cpp
// Extends duration for ALL waypoints from index 1 onwards
for (Eigen::Index time_stretch_idx = 1; time_stretch_idx < static_cast<Eigen::Index>(num_waypoints);
     ++time_stretch_idx)
{
    const double duration_from_previous =
        duration_extension_factor * original_duration_from_previous(time_stretch_idx);
    new_duration_from_previous(time_stretch_idx) = duration_from_previous;
    // ... updates all waypoints
}
```

**Impact:**
- Tesseract extends entire trajectory unnecessarily, making it slower than needed
- More iterations required to converge
- Less efficient smoothing process

---

### 4. **Trajectory Unwinding for Angle Wrapping**

**Status:** Missing in Tesseract

**MoveIt2 Code:**
```cpp
// This lib does not work properly when angles wrap, so we need to unwind the path first
trajectory.unwind();
```

**Location:** `ruckig_traj_smoothing.cpp:239`

**Impact:**
Ruckig doesn't handle angular joint wrapping (e.g., when a joint goes from +179° to -179°). Without unwinding, trajectories with wrapped angles may fail or produce incorrect motions.

**Tesseract Status:**
Does not call `unwind()` before smoothing. This could cause issues with revolute joints that cross the ±π boundary.

---

### 5. **Input Clamping Improvements** (Added to MoveIt2: May 4, 2022)

**Status:** Partially implemented in Tesseract

**MoveIt2 Implementation:**
Clamps both current AND target velocities/accelerations in `getNextRuckigInput()`:
```cpp
// Clamp current state
ruckig_input.current_velocity.at(joint) =
    std::clamp(ruckig_input.current_velocity.at(joint),
               -ruckig_input.max_velocity.at(joint),
               ruckig_input.max_velocity.at(joint));

// Clamp target state
ruckig_input.target_velocity.at(joint) =
    std::clamp(ruckig_input.target_velocity.at(joint),
               -ruckig_input.max_velocity.at(joint),
               ruckig_input.max_velocity.at(joint));
```

**Tesseract Implementation:**
Only clamps in `getNextRuckigInput()` but uses different approach:
```cpp
current_velocity = current_velocity.array().min(max_velocity.array()).max((-1.0 * max_velocity).array());
```

**Difference:**
MoveIt2 clamps each joint individually using `std::clamp()`, which is clearer and handles symmetric bounds properly. Tesseract's Eigen array operations achieve similar results but are less explicit.

---

### 6. **Algorithm Structure: calculate() vs update()**

**Status:** Different API usage

**MoveIt2:**
Uses `ruckig.calculate()` which computes the entire trajectory segment at once:
```cpp
ruckig_result = ruckig.calculate(ruckig_input, ruckig_output);
```

**Tesseract:**
Uses `ruckig.update()` which is more iterative:
```cpp
ruckig_result = ruckig_ptr->update(ruckig_input, ruckig_output);
```

**Impact:**
- `calculate()` returns a `Trajectory` object with full trajectory information
- `update()` is typically used for online/streaming mode
- MoveIt2's approach is more appropriate for offline trajectory smoothing
- Both work, but `calculate()` provides better trajectory introspection

---

### 7. **Error Handling and Logging**

**MoveIt2:**
- Uses structured ROS2 logging (`RCLCPP_ERROR`, `RCLCPP_WARN`)
- Better error messages with context
- Includes logger name for filtering

**Tesseract:**
- Uses console_bridge logging (`CONSOLE_BRIDGE_logError`)
- Less structured logging
- Simpler error messages

**Impact:**
Minor - mainly affects debugging and user experience.

---

### 8. **Ruckig Result Code Interpretation**

**MoveIt2:**
Explicitly handles the difference between `Result::Working` and `Result::Finished`:
```cpp
// The difference between Result::Working and Result::Finished is that Finished can be reached in one
// Ruckig timestep (constructor parameter). Both are acceptable for trajectories.
```

**Tesseract:**
Only accepts `Result::Finished` as success (see issue #2 above).

---

## Commit History Analysis

### Key MoveIt2 Commits (Chronological)

1. **Sept 16, 2021** - Original Ruckig implementation
2. **May 4, 2022** - Input clamping and waypoint reuse improvements
3. **May 10, 2022** - Custom limits support via unordered_map parameters
4. **Dec 20, 2022** - Adaptive batch sizes (later removed)
5. **Mar 10, 2023** - **[BUG FIX]** Termination condition fix
6. **Mar 16, 2023** - **[OPTIMIZATION]** Duration extension optimization
7. **Mar 27, 2023** - **[FEATURE]** Overshoot mitigation
8. **Sept 2023** - Code formatting updates
9. **Mar 2024** - Unified logging migration
10. **Nov 24, 2024** - Template compatibility fix for checkOvershoot

---

## Recommendations for Tesseract

### High Priority (Bug Fixes)

1. **Fix termination condition** - Accept both `Result::Working` and `Result::Finished`
2. **Add trajectory unwinding** - Call unwind before smoothing to handle angle wrapping
3. **Optimize duration extension** - Only extend failed segments, not entire trajectory

### Medium Priority (Features)

4. **Add overshoot mitigation** - Implement `checkOvershoot()` with optional parameter
5. **Review algorithm API** - Consider switching from `update()` to `calculate()`

### Low Priority (Improvements)

6. **Improve logging** - Add more context to error messages
7. **Code cleanup** - Align with MoveIt2's latest code structure

---

## Code Compatibility Notes

### Shared Origins
Both implementations originate from the same PickNik Robotics BSD 3-Clause licensed code (authors: Jack Center, Wyatt Rees, Andy Zelenak, Stephanie Eng).

### Tesseract Additions
- Levi Armstrong added Tesseract integration
- Profile-based configuration system
- Different data structures (InstructionsTrajectory vs RobotTrajectory)

### Architecture Differences
- MoveIt2: Part of `trajectory_processing` namespace, works with `robot_trajectory::RobotTrajectory`
- Tesseract: Part of `tesseract_planning` namespace, works with `CompositeInstruction` and `InstructionsTrajectory`

---

## File References

### Tesseract Files
- `tesseract_time_parameterization/ruckig/src/ruckig_trajectory_smoothing.cpp`
- `tesseract_time_parameterization/ruckig/include/tesseract_time_parameterization/ruckig/ruckig_trajectory_smoothing.h`

### MoveIt2 Files
- `moveit_core/trajectory_processing/src/ruckig_traj_smoothing.cpp`
- `moveit_core/trajectory_processing/include/moveit/trajectory_processing/ruckig_traj_smoothing.hpp`

### MoveIt2 Repository
- https://github.com/moveit/moveit2/tree/main/moveit_core/trajectory_processing

---

## Conclusion

Tesseract's Ruckig implementation is functional but missing approximately 2 years of bug fixes and improvements from MoveIt2. The most critical issues are:

1. **Termination condition bug** - May cause unnecessary duration extensions
2. **Missing trajectory unwinding** - Can fail with angle wrapping
3. **Inefficient duration extension** - Slows entire trajectory when only one segment fails

Implementing these fixes would significantly improve Tesseract's trajectory smoothing reliability and performance.
