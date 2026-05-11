# Additional Improvements Tesseract TOTG May Be Missing from MoveIt2

Based on comprehensive analysis of MoveIt/MoveIt2 commits and PRs from 2020-2025.

> ⚠️ **Review update — 2026-05-11:** Additional findings have been added at the
> bottom of this document (§9). MoveIt2 had no commits to TOTG cpp/hpp between
> the original analysis and this review. File paths/namespaces are stale; current
> source is at `time_parameterization/totg/...`, namespace
> `tesseract::time_parameterization`. See `REVIEW_UPDATE_2026-05.md` for the
> consolidated review.

---

## 1. ✅ ALREADY HAS: Ruckig Trajectory Smoothing

**Status:** ✅ **Tesseract already has this**

**Evidence:** Found in `/tesseract_time_parameterization/ruckig/include/tesseract_time_parameterization/ruckig/ruckig_trajectory_smoothing.h` (dated July 27, 2022)

**MoveIt2 Implementation:** [PR #571](https://github.com/moveit/moveit2/pull/571) - Ruckig trajectory_processing plugin

**What it provides:**
- Jerk limits support (TOTG does not support jerk limits)
- Nonzero initial/final velocities and accelerations
- Smoother trajectories with bounded jerk

---

## 2. ❌ MISSING: Torque Limits Support in TOTG

**Status:** ❌ **Tesseract does NOT have this**

**MoveIt Implementation:** [PR #3412](https://github.com/moveit/moveit/pull/3412) and [PR #3427](https://github.com/moveit/moveit/pull/3427) (Merged May 2023)

**What it provides:**
```cpp
// MoveIt has this method:
bool computeTimeStampsWithTorqueLimits(
    robot_trajectory::RobotTrajectory& trajectory,
    const std::unordered_map<std::string, double>& torque_limits,
    const std::vector<geometry_msgs::Wrench>& external_wrenches = {},
    double max_velocity_scaling_factor = 1.0,
    double max_acceleration_scaling_factor = 1.0) const;
```

**Algorithm:**
1. Apply standard TOTG with velocity/acceleration constraints
2. Run forward dynamics to detect torque limit violations
3. Iteratively reduce acceleration limits for violated joints
4. Repeat until torque compliant

**Benefits:**
- Respects actual motor torque capabilities
- Uses URDF inertia model for dynamics
- Prevents motor saturation and overheating
- More realistic trajectory execution

**Tesseract Status:**
- Current profile only has velocity and acceleration limits
- No torque limit fields in `TimeOptimalTrajectoryGenerationCompositeProfile`
- Would require significant implementation effort

---

## 3. ⚠️ PARTIALLY MISSING: Configurable Minimum Velocity/Acceleration Limits

**Status:** ⚠️ **May have different defaults**

**MoveIt Changes:** [PR #2937](https://github.com/moveit/moveit/pull/2937) (Merged 2021)

**What changed:**
- Reduced minimum max velocity from 0.01 to 0.001 rad/s (or m/s)
- Allows slower, more precise movements
- Better support for very slow joints (e.g., grippers, small actuators)

**Tesseract Status:**
- Need to verify minimum limit enforcement
- No obvious hard-coded minimums in current code
- Profile allows arbitrary limits via `override_limits`

**Recommendation:** Verify Tesseract doesn't enforce minimum limits that are too high

---

## 4. ⚠️ DIFFERENT: Limit Validation Behavior

**Status:** ⚠️ **Different philosophies**

**MoveIt2 Change:** [PR #1794](https://github.com/ros-planning/moveit2/pull/1794)

**What changed:**
- **Old behavior:** Print warning if limits missing, use defaults, continue
- **New behavior:** Print error if limits missing, **return false** (fail hard)

**Rationale:** "A warning was printed but it was easy to overlook... This patch helps users to avoid an unexpected behavior of the robot moving very slow"

**Tesseract Behavior:**
Looking at lines 97-108 in `time_optimal_trajectory_generation.cpp`:
```cpp
// Validate limits
if (velocity_limits.rows() != acceleration_limits.rows())
{
  CONSOLE_BRIDGE_logError("Invalid velocity or acceleration specified...");
}
```
- Logs error but **continues execution** (doesn't return false)
- Could lead to undefined behavior

**Recommendation:** Consider failing hard when limits are invalid, like MoveIt2

---

## 5. ✅ ALREADY HAS: Configurable Path Tolerance and Min Angle Change

**Status:** ✅ **Tesseract already has this**

**MoveIt Feature:** [PR #2185](https://github.com/moveit/moveit/pull/2185) - Parameterize input trajectory density

**Evidence in Tesseract:**
```cpp
// In TimeOptimalTrajectoryGenerationCompositeProfile.h:
double path_tolerance{ 0.1 };      // Line 61
double min_angle_change{ 0.001 };   // Line 64
```

**What it provides:**
- Control downsampling density for long/dense trajectories
- Higher `min_angle_change` = fewer points = faster computation
- Users can tune performance vs. accuracy trade-off

---

## 6. ⚠️ MISSING: Polymorphic TimeParameterization Design Pattern

**Status:** ⚠️ **Different architecture**

**MoveIt Enhancement:** Made `TimeParameterization` classes polymorphic (Version 1.1.8, 2022)

**What it provides:**
- Better extensibility via inheritance
- Easier to add custom time parameterization algorithms
- Plugin-based architecture

**Tesseract Status:**
- Has base class `TimeParameterization` (line 32 in profile header)
- Has multiple implementations: TOTG, Ruckig, ISP, KDL
- Already appears polymorphic

**Conclusion:** ✅ Tesseract already has this design

---

## 7. ⚠️ UNCLEAR: Multiple `computeTimeStamps` Overloads

**Status:** ⚠️ **Need to verify**

**MoveIt Feature:** Two overloads of `computeTimeStamps()`

**Overload 1:** Uses robot model's built-in limits
```cpp
bool computeTimeStamps(robot_trajectory::RobotTrajectory& trajectory,
                      const double max_velocity_scaling_factor = 1.0,
                      const double max_acceleration_scaling_factor = 1.0) const;
```

**Overload 2:** Uses custom per-joint limits
```cpp
bool computeTimeStamps(robot_trajectory::RobotTrajectory& trajectory,
                      const std::unordered_map<std::string, double>& velocity_limits,
                      const std::unordered_map<std::string, double>& acceleration_limits,
                      const double max_velocity_scaling_factor = 1.0,
                      const double max_acceleration_scaling_factor = 1.0) const;
```

**Tesseract Status:**
- Has single `compute()` method
- Supports limit override via profile's `override_limits` flag
- Achieves same goal with different API

**Conclusion:** ✅ Equivalent functionality, different API design

---

## 8. ✅ ALREADY FIXED: Better Antiparallel Vector Detection

**Status:** ⚠️ **Partially implemented** (see earlier analysis)

**MoveIt Fix:** Part of various numerical stability improvements

**Recommendation from earlier analysis:**
```cpp
// Add explicit antiparallel check in CircularPathSegment
const double start_dot_end = start_direction.dot(end_direction);
if ((start_direction - end_direction).norm() < 0.000001 ||
    start_dot_end > 0.999999 || start_dot_end < -0.999999)
{
  // Early exit with zero-length segment
}
```

---

## Summary Table

| Feature | MoveIt2 Status | Tesseract Status | Priority | Effort |
|---------|---------------|------------------|----------|--------|
| **Ruckig smoothing** | ✅ Has | ✅ **Has** | N/A | N/A |
| **Torque limits** | ✅ Has (2023) | ❌ **Missing** | 🔴 HIGH | 🔴 HIGH |
| **Minimum limit reduction** | ✅ Has (2021) | ⚠️ Unknown | 🟡 MEDIUM | 🟢 LOW |
| **Strict limit validation** | ✅ Has (PR #1794) | ⚠️ Lenient | 🟡 MEDIUM | 🟢 LOW |
| **Configurable density** | ✅ Has (PR #2185) | ✅ **Has** | N/A | N/A |
| **Polymorphic design** | ✅ Has (2022) | ✅ **Has** | N/A | N/A |
| **Multiple overloads** | ✅ Has | ✅ **Equivalent** | N/A | N/A |
| **Antiparallel detection** | ✅ Has | ⚠️ **Partial** | 🟢 LOW | 🟢 LOW |

---

## Detailed Recommendations

### Priority 1: Consider Adding Torque Limits Support

**Difficulty:** 🔴 High (Major feature)

**Benefits:**
- More realistic trajectory execution
- Prevents motor damage from overcurrent
- Better utilization of robot capabilities
- Required for heavy payload applications

**Implementation approach:**
1. Add torque limit fields to profile:
   ```cpp
   Eigen::MatrixX2d torque_limits;
   bool use_torque_limits{ false };
   ```
2. Implement forward dynamics using URDF inertia
3. Add iterative refinement loop
4. Consider external wrenches support

**References:**
- [MoveIt PR #3412](https://github.com/moveit/moveit/pull/3412)
- [MoveIt PR #3427](https://github.com/moveit/moveit/pull/3427)

---

### Priority 2: Improve Antiparallel Vector Handling

**Difficulty:** 🟢 Low (Minor fix)

**Current issue:** Only checks parallel (same direction), not antiparallel (opposite direction)

**Fix:** Add explicit dot product check in `CircularPathSegment` constructor (line ~237):
```cpp
const double start_dot_end = start_direction.dot(end_direction);

// Check BOTH parallel AND antiparallel
if ((start_direction - end_direction).norm() < 0.000001 ||
    start_dot_end > 0.999999 || start_dot_end < -0.999999)
{
  length_ = 0.0;
  radius = 1.0;
  center = intersection;
  x = Eigen::VectorXd::Zero(start.size());
  y = Eigen::VectorXd::Zero(start.size());
  return;
}
```

---

### Priority 3: Consider Stricter Limit Validation

**Difficulty:** 🟢 Low (Simple check)

**Current behavior:** Logs error, continues execution

**Proposed behavior:** Return `false` when limits invalid

**Change in `time_optimal_trajectory_generation.cpp` ~line 105:**
```cpp
// Validate limits
if (velocity_limits.rows() != acceleration_limits.rows())
{
  CONSOLE_BRIDGE_logError("Invalid velocity or acceleration specified...");
  return false;  // ← ADD THIS
}
```

**Rationale:** Fail fast, don't let invalid configuration cause undefined behavior

---

### Priority 4: Verify Minimum Limit Handling

**Difficulty:** 🟢 Low (Investigation)

**Action:** Check if Tesseract enforces minimum velocity/acceleration limits

**If found:** Consider reducing minimums like MoveIt did (0.01 → 0.001)

---

## Conclusion

**Tesseract TOTG is very well-maintained** and includes most modern MoveIt2 improvements:
- ✅ Ruckig support (jerk limits)
- ✅ Configurable density parameters
- ✅ Superior numerical stability in some areas
- ✅ Zero endpoint velocities (safe concatenation)
- ✅ Polymorphic design

**Main gap:** Torque limits support (significant feature, high effort)

**Quick wins:**
1. Antiparallel vector detection (5-10 lines of code)
2. Stricter limit validation (1 line of code)
3. Verify minimum limit handling

---

## References

### MoveIt Pull Requests
- [PR #571 - Ruckig trajectory smoothing](https://github.com/moveit/moveit2/pull/571)
- [PR #809 - Add TOTG plugin](https://github.com/moveit/moveit/pull/809)
- [PR #1218 - Make TOTG default](https://github.com/moveit/moveit2/pull/1218)
- [PR #1729 - Fix invalid accelerations](https://github.com/moveit/moveit/pull/1729)
- [PR #1794 - Require velocity/acceleration limits](https://github.com/ros-planning/moveit2/pull/1794)
- [PR #1861 - Fix segfault in TOTG](https://github.com/moveit/moveit/pull/1861)
- [PR #2054 - Single-waypoint trajectories](https://github.com/moveit/moveit/pull/2054)
- [PR #2185 - Parameterize trajectory density](https://github.com/moveit/moveit/pull/2185)
- [PR #2882 - Readability improvements](https://github.com/moveit/moveit/pull/2882)
- [PR #2937 - Reduce minimum limits](https://github.com/moveit/moveit/pull/2937)
- [PR #2957 - API stress tests, fix undefined behavior](https://github.com/moveit/moveit/pull/2957)
- [PR #3412 - TOTG with torque limits](https://github.com/moveit/moveit/pull/3412)
- [PR #3427 - Fixup TOTG torque limits](https://github.com/moveit/moveit/pull/3427)

### MoveIt Issues
- [Issue #1665 - Invalid accelerations](https://github.com/moveit/moveit/issues/1665)
- [Issue #2495 - TOTG may fail returning to start](https://github.com/moveit/moveit/issues/2495)
- [Issue #2741 - Blend radius duplicate timestamps](https://github.com/moveit/moveit2/issues/2741)
- [Issue #3014 - Endpoint velocity changes](https://github.com/moveit/moveit2/issues/3014)
- [Issue #3504 - Acceleration limits not loaded](https://github.com/moveit/moveit2/issues/3504)

### Documentation
- [MoveIt Time Parameterization Tutorial](https://moveit.picknik.ai/humble/doc/examples/time_parameterization/time_parameterization_tutorial.html)
- [MoveIt Trajectory Processing](https://moveit.picknik.ai/main/doc/concepts/trajectory_processing.html)
- [Ruckig Jerk-limited Smoothing](https://discourse.openrobotics.org/t/jerk-limited-trajectory-smoothing-in-moveit2/25089)

---

## 9. Additional findings — added 2026-05-11

These items were discovered during the second-pass code review and are not
covered by the sections above. Full context in `REVIEW_UPDATE_2026-05.md`.

### 9a. Exposure to moveit2#3565 — huge accelerations from tiny `time_step`

**Severity:** 🟡 medium — rare, but produces accelerations many orders of
magnitude over the configured limits when it fires.

**Location:** `getAcceleration` at
`time_parameterization/totg/src/time_optimal_trajectory_generation.cpp:1079-1086`.

```cpp
Eigen::VectorXd Trajectory::getAcceleration(const PathData& data) const
{
  Eigen::VectorXd path_acc =
      (path_.getTangent(data.path_pos) * data.path_vel - path_.getTangent(data.prev_path_pos) * data.prev_path_vel);
  double time_step = data.time - data.prev_time;
  if (time_step > 0.0)
    path_acc /= time_step;
  return path_acc;
}
```

`if (time_step > 0.0)` protects against exact zero, not against `1e-15`. Two ways
`time_step` can be tiny:
1. Adjacent steps in `trajectory_` end up spatially very close after the
   bisection in `integrateForward`, so the constructor's timing pass at line 487
   yields a very small `dt = Δpos / avg_vel`.
2. `getPathData` queries near a segment boundary give `data.time − data.prev_time`
   approaching zero.

The `1e-8` floor in `assignData` (lines 960-961, 972-973) does not protect this
code path — it only guards monotonicity at the **outer waypoint** level.

**MoveIt2 status:** open as
[moveit2#3565](https://github.com/moveit/moveit2/issues/3565), no fix merged.

### 9b. `getTime` divides by zero in coast regions

**Severity:** 🟡 medium — fires whenever `assignData` calls `getTime` on a
coast-region position; coast regions are common in TOTG output.

**Location:** `time_optimal_trajectory_generation.cpp:1057-1067`.

```cpp
double time_step = it->time_ - previous->time_;
const double acceleration =
    2.0 * (it->path_pos_ - previous->path_pos_ - time_step * previous->path_vel_) / (time_step * time_step);

const double a = 0.5 * acceleration;
...
const double dt = (-b + e) / (2.0 * a);
assert(!(dt < 0));
```

If two adjacent `trajectory_` steps have equal `path_vel_` (a coast region),
`acceleration = 0`, `a = 0`, and the final divide produces `±∞`. The assertion
does not catch `+∞`.

Fix shape: linear fallback `dt = (pos − previous->path_pos_) / previous->path_vel_`
when `|a|` is below some epsilon.

### 9c. Inconsistent zero-detection in `getAccelerationMaxPathVelocity`

**Severity:** 🟢 low.

**Location:** `time_optimal_trajectory_generation.cpp:870-900`.

`getMinMaxPathAcceleration` (line 854) uses
`tesseract::common::almostEqualRelativeAndAbs(config_deriv[i], 0.0, eps)` to
guard divisions, but `getAccelerationMaxPathVelocity` two functions below uses
bare `config_deriv[i] != 0.0` and `a_ij != 0.0` (lines 877, 881, 884) for the
same purpose. A `config_deriv[i]` of `1e-300` passes `!= 0.0` then divides into
~`1e+300` at line 887.

Fix: make the two functions consistent.

### 9d. Copy-paste error in error messages

**Severity:** 🟢 trivial.

**Location:** lines 99 and 103.

Both throw `std::runtime_error("IterativeSplineParameterization, velocity scale factor must be greater than zero!")` from inside TOTG code. Line 103 is about
*acceleration*, not velocity. Misleading when debugging.

### 9e. Strengthened severity — antiparallel handling

The original document framed the antiparallel issue as "potential bug." The
actual consequence (verified in `REVIEW_UPDATE_2026-05.md` §6) is that
`CircularPathSegment` produces `center = NaN`, `x = NaN`, and `getConfig`
returns NaN for any input, and the surrounding `Path::Path` code silently
swallows that NaN. The `acos` clamp only prevents one NaN source; the geometry
itself is broken. The dummy-joint workaround for Issue #27 is what currently
keeps this from manifesting in production.

The proposed `start_dot_end < -0.999999` early-exit is therefore a precondition
for safely removing the dummy-joint workaround — higher leverage than the
original document suggested.
