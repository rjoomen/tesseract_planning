# Ruckig Implementation Comparison: Tesseract vs MoveIt2

## Executive Summary

The Tesseract Ruckig implementation is based on MoveIt2's code but is missing several critical bug fixes and improvements that were added to MoveIt2 between 2022-2023. This document outlines the key differences and missing features.

## Key Findings Summary

1. **Architectural Mismatch:** Tesseract uses Ruckig's `update()` API (designed for real-time control loops) instead of `calculate()` (designed for offline trajectory planning). This prevents implementation of overshoot mitigation.

2. **Missing March 2023 Bug Fixes:** Three critical bugs that were fixed in MoveIt2 are still present in Tesseract.

3. **Based on Outdated Code:** Despite being implemented in April 2025, Tesseract's code is based on pre-March 2023 MoveIt2 code.

4. **No Post-2023 Impact:** Good news - no algorithmic improvements were made to MoveIt2 after March 2023, so Tesseract only needs the 2023 fixes.

---

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

**Status:** Different API usage - SIGNIFICANT ARCHITECTURAL DIFFERENCE

**Ruckig API Background:**
- **`calculate()`** - Offline trajectory generation, returns a complete `Trajectory` object
  - Best for pre-computing trajectories when target is known
  - Provides full trajectory with sampling capability via `at_time()`
  - Doesn't require control cycle in constructor

- **`update()`** - Real-time/online trajectory generation
  - Designed for control loops, updates output parameters incrementally
  - Requires control cycle (timestep) specified during initialization
  - Allows dynamic target adjustments mid-trajectory

**MoveIt2 Implementation:**
Uses `calculate()` for offline trajectory smoothing:
```cpp
ruckig::Ruckig<ruckig::DynamicDOFs> ruckig(num_dof, trajectory.getAverageSegmentDuration());
ruckig_result = ruckig.calculate(ruckig_input, ruckig_output);

// Then uses the trajectory object:
trajectory.setWayPointDurationFromPrevious(waypoint_idx + 1, ruckig_output.get_duration());
checkOvershoot(ruckig_output, ...);  // Samples trajectory at multiple time points
```

**Tesseract Implementation:**
Uses `update()` with explicit timestep:
```cpp
double timestep = original_duration_from_previous.sum() / static_cast<double>(num_waypoints - 1);
auto ruckig_ptr = std::make_unique<ruckig::Ruckig<ruckig::DynamicDOFs>>(dof, timestep);
ruckig_result = ruckig_ptr->update(ruckig_input, ruckig_output);
// ruckig_output is just OutputParameter with new_position/velocity/acceleration
// No Trajectory object, cannot sample at arbitrary time points
```

**Note:** Tesseract's code has an `#ifdef WITH_ONLINE_CLIENT` block (lines 59-123) that uses `calculate()`, but this is **not compiled** (flag is never defined in CMakeLists.txt). The actual production code uses `update()`.

**Impact:**

1. **Architectural Mismatch:**
   - Tesseract uses the **online/real-time API** for **offline trajectory smoothing**
   - MoveIt2 correctly uses the **offline API** for offline smoothing
   - Both work, but `update()` is designed for a different use case

2. **Overshoot Detection Impossible in Tesseract:**
   - MoveIt2's `checkOvershoot()` requires `Trajectory.at_time()` to sample the trajectory
   - Tesseract's `update()` only returns `OutputParameter` (single state, not full trajectory)
   - **Cannot implement overshoot mitigation without switching to `calculate()`**

3. **Missing Trajectory Introspection:**
   - `calculate()` provides `get_duration()` method
   - `calculate()` allows sampling trajectory at any time point
   - `update()` only gives next state, no trajectory analysis capability

4. **Performance:**
   - `update()` may have overhead from control cycle logic designed for real-time use
   - `calculate()` is optimized for offline computation

**Conclusion:** Tesseract's use of `update()` is functional but architecturally incorrect for offline trajectory smoothing. Switching to `calculate()` would:
- Enable overshoot mitigation implementation
- Better match the use case (offline planning vs real-time control)
- Provide cleaner access to trajectory duration and properties

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

**Core Algorithm Development (2021-2023):**
1. **Sept 16, 2021** - Original Ruckig implementation
2. **May 4, 2022** - Input clamping and waypoint reuse improvements
3. **May 10, 2022** - Custom limits support via unordered_map parameters
4. **Dec 20, 2022** - Adaptive batch sizes (later removed)
5. **Mar 10, 2023** - **[BUG FIX]** Termination condition fix
6. **Mar 16, 2023** - **[OPTIMIZATION]** Duration extension optimization
7. **Mar 27, 2023** - **[FEATURE]** Overshoot mitigation
8. **Sept 2023** - Code formatting updates

**Maintenance & Code Quality (2024-2025):**
9. **Mar 15, 2024** - Unified logging names across moveit_core
10. **Jun 2024** - Added utility functions for joint limits and trajectory messages
11. **Aug 2024** (PR #2956) - Ruckig plugin for MoveIt Servo (real-time smoothing)
12. **Nov 6, 2024** - Fix createTrajectoryMessage with improved sampling rate handling
13. **Nov 29, 2024** - Header refactoring (.h to .hpp extensions)
14. **Jan 9, 2025** - Added const specifiers to trajectory methods
15. **Nov 24, 2025** - Template compatibility fix for checkOvershoot

**Note:** After March 2023, all changes to the offline trajectory smoothing algorithm have been maintenance-focused (code quality, logging, const-correctness, template fixes). No algorithmic improvements or bug fixes to the core smoothing logic have been made since March 2023.

### Tesseract Ruckig Commits (Chronological)

**Initial Implementation:**
1. **Apr 19, 2025** (f6cadbb) - Initial Ruckig implementation added by Levi Armstrong
   - Based on MoveIt2's code with PickNik Robotics BSD 3-Clause license
   - Authors credited: Jack Center, Wyatt Rees, Andy Zelenak, Stephanie Eng, Levi Armstrong
   - Added 654 lines across 5 files (header, implementation, tests, CMake)
   - **Already contained the termination condition bug** (only checks `Result::Finished`)

**Subsequent Updates:**
2. **May 17, 2025** (9c7252d) - Update time parameterization interface to leverage profiles
   - Major refactoring to use profile-based configuration
   - Changed from constructor parameters to `RuckigTrajectorySmoothingCompositeProfile`
   - Modified compute() interface to use `CompositeInstruction` and `ProfileDictionary`
   - 42 insertions, 33 deletions in main implementation file
   - **Termination bug persists** (still only checks `Result::Finished`)

3. **Nov 24, 2025** (a257de5) - Switch to Cereal for serialization (#681)
   - Added Cereal serialization support for profiles
   - Added `cereal_serialization.h` and `cereal_serialization.cpp`
   - Updated profile serialization methods

4. **Nov 29, 2025** (f9b5ce0) - Improve serialization coverage
   - Added more serialization tests

5. **Nov 29, 2025** (5ceb28f) - Improve time parameterization coverage
   - Added 12 lines of test coverage

6. **Nov 29, 2025** (d75def8) - Remove getStaticKey from profiles
   - Cleanup of profile interface

7. **Nov 29, 2025** (39b3e53) - Cleanup doxygen file headers
   - Documentation improvements

### MoveIt2 Post-2023 Improvements

**Question: Did MoveIt2 have algorithmic improvements after 2023?**

**Answer: No.** After the critical March 2023 bug fixes, all subsequent changes (2024-2025) have been **maintenance and code quality improvements only**:

**2024 Changes:**
- **Logging improvements** - Unified logger names for better debugging
- **API additions** - Utility functions for joint limits and trajectory messages
- **Bug fix** - createTrajectoryMessage sampling rate calculation using std::ceil
- **Header standardization** - Converted .h to .hpp extensions
- **Real-time feature** - Added Ruckig plugin for MoveIt Servo (different use case - online smoothing, not offline trajectory planning)

**2025 Changes:**
- **Code quality** - Added const qualifiers to member functions
- **Template fix** - Corrected checkOvershoot template parameter (removed unnecessary StandardVector)

**Important:** None of these post-2023 changes affect the core trajectory smoothing algorithm or fix any additional bugs. The last algorithmic improvements were:
- **March 10, 2023** - Termination condition fix
- **March 16, 2023** - Duration extension optimization
- **March 27, 2023** - Overshoot mitigation

**Conclusion:** Tesseract is missing the March 2023 fixes, but hasn't missed any algorithmic improvements from 2024-2025 (because there weren't any). The post-2023 changes are nice-to-haves but not critical for functionality.

**Known Issues:**
- Issue #3008 (open since Sept 2024) - Ruckig smoothing plugin breaks MoveIt Servo demos
  - This affects the Servo plugin, not the offline trajectory smoothing that Tesseract uses

---

### Key Observations from Tesseract History

**Initial Code Source:**
- Tesseract's initial implementation (April 2025) was based on MoveIt2's code
- However, it appears to be based on an **early version** of MoveIt2's implementation (likely pre-March 2023)
- The code already contained the termination condition bug that MoveIt2 fixed in March 2023

**No Bug Fixes Applied:**
- **Zero bug fixes** from MoveIt2's 2023 improvements have been backported to Tesseract
- All Tesseract commits focus on:
  - Tesseract-specific integration (profiles, serialization)
  - Test coverage improvements
  - Code cleanup and documentation
- **None** address the core algorithmic bugs identified in MoveIt2

**Development Focus:**
- Tesseract development has focused on integration rather than algorithm improvements
- Profile-based configuration is a Tesseract-specific enhancement
- No evidence of tracking or incorporating MoveIt2's subsequent bug fixes

**Timeline Gap:**
- MoveIt2's critical bug fixes: **March 2023**
- Tesseract's initial implementation: **April 2025** (2 years later!)
- Despite the 2-year gap, Tesseract used pre-bugfix code as the base

---

## Recommendations for Tesseract

### High Priority (Bug Fixes)

1. **Fix termination condition** - Accept both `Result::Working` and `Result::Finished`
2. **Add trajectory unwinding** - Call unwind before smoothing to handle angle wrapping
3. **Optimize duration extension** - Only extend failed segments, not entire trajectory

### Medium Priority (Features)

4. **Switch from `update()` to `calculate()`** - Required for overshoot mitigation and architecturally correct
   - Current `update()` API is designed for real-time control, not offline smoothing
   - `calculate()` returns Trajectory object needed for overshoot detection
   - Must be done before implementing overshoot mitigation

5. **Add overshoot mitigation** - Implement `checkOvershoot()` with optional parameter
   - **Prerequisite:** Must switch to `calculate()` first (item #4)
   - Requires `Trajectory.at_time()` method to sample trajectory

### Low Priority (Improvements)

6. **Improve logging** - Add more context to error messages
7. **Code cleanup** - Align with MoveIt2's latest code structure
8. **Consider template fixes** - Apply Nov 2025 checkOvershoot template parameter fix (if implementing overshoot mitigation)
9. **Add const qualifiers** - Follow MoveIt2's const-correctness improvements

**Note:** Low priority items are from MoveIt2's 2024-2025 maintenance work - they improve code quality but don't affect functionality.

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

## Timeline Comparison

```
MoveIt2 Timeline:
═══════════════════════════════════════════════════════════════════════════════
Sept 2021    Initial Ruckig implementation
    |
May 2022     Input clamping improvements + Custom limits support
    |
Dec 2022     Batch size adjustments
    |
Mar 2023     🔴 CRITICAL BUG FIXES 🔴
    |        - Termination condition fix (Mar 10)
    |        - Duration extension optimization (Mar 16)
    |        - Overshoot mitigation (Mar 27)
    |
Sept 2023    Code formatting
    |
Mar 2024     Unified logging
    |
Nov 2024     Template compatibility fix
    |
    ▼

Tesseract Timeline:
═══════════════════════════════════════════════════════════════════════════════
                                            Apr 2025    Initial implementation
                                                |       (based on pre-Mar 2023 code!)
                                                |       ❌ Contains termination bug
                                                |       ❌ Missing overshoot mitigation
                                                |       ❌ Inefficient duration extension
                                                |
                                            May 2025    Profile-based interface
                                                |       ❌ Bug still present
                                                |
                                            Nov 2025    Serialization updates
                                                |       ❌ Bug still present
                                                ▼

Key Issue: Tesseract implemented Ruckig in 2025 but based it on MoveIt2's
          PRE-2023 code, missing 2 years of critical bug fixes!
```

---

## Conclusion

Tesseract's Ruckig implementation is functional but missing approximately 2 years of bug fixes and improvements from MoveIt2. The most critical issues are:

1. **Termination condition bug** - May cause unnecessary duration extensions
2. **Missing trajectory unwinding** - Can fail with angle wrapping
3. **Inefficient duration extension** - Slows entire trajectory when only one segment fails

### Critical Finding

Despite being implemented in **April 2025** (2 years after MoveIt2's March 2023 bug fixes), Tesseract's implementation appears to be based on **pre-March 2023** MoveIt2 code. This means:

- ❌ The code was already outdated when it was added to Tesseract
- ❌ Known bugs were inadvertently introduced into Tesseract
- ❌ No subsequent updates have addressed these issues

### Recommendation

**High Priority:** Update Tesseract's Ruckig implementation to incorporate MoveIt2's **March 2023 bug fixes**. This is not about adding new features, but fixing known bugs that have proven solutions.

**Good News:** Tesseract hasn't missed any post-2023 algorithmic improvements (there haven't been any). All MoveIt2 changes from 2024-2025 are maintenance/code quality improvements that don't affect core functionality.

**Focus Areas:**
1. **March 2023 fixes are CRITICAL** - These fix actual bugs
2. **Post-2023 changes are OPTIONAL** - These are nice-to-haves for code quality

Implementing the March 2023 fixes would significantly improve Tesseract's trajectory smoothing reliability and performance.
