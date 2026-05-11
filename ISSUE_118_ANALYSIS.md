# Issue [#118](https://github.com/tesseract-robotics/tesseract_planning/issues/118) Analysis: Unused Velocity/Acceleration Scaling Factors

**Issue:** [tesseract-robotics/tesseract_planning#118](https://github.com/tesseract-robotics/tesseract_planning/issues/118)
**Opened:** September 22, 2021
**Status:** ⚠️ **OPEN but APPEARS FIXED in current code**
**Severity:** 🟡 **MEDIUM** (if still broken) / 🟢 **LOW** (if fixed but not closed)

> 📌 **Review update — 2026-05-11:** Verified — scaling factors are applied at
> `time_parameterization/totg/src/time_optimal_trajectory_generation.cpp:168-173`.
> Conclusions in this document are correct. File path updated for the recent
> repo restructure.

---

## Problem Description (Original Issue)

### Issue Summary
> "Time optimal trajectory has unused variables velocity_scaling_factor and acceleration_scaling_factory"

**Reporter:** Levi-Armstrong

**Problem:**
The TOTG implementation had scaling factor parameters (`max_velocity_scaling_factor` and `max_acceleration_scaling_factor`) in the profile that were validated but **not actually used** when computing the trajectory.

### Matthew Powelson's Response

**Original Intent:**
- Scaling factors should be passed to TimeOptimalTrajectoryGeneration for internal handling
- When set per MoveInstruction, TOTG should use default value of 1.0, with rescaling applied afterward

**Admission:**
> "looking at it now it looks like scaling factors inside TimeOptimalTrajectoryGeneration aren't doing anything. So that should probably be fixed."

---

## Current Code Analysis (January 2026)

### Scaling Factors ARE Now Being Used ✅

**Location:** Lines 168-172 in `time_optimal_trajectory_generation.cpp`

```cpp
// Line 168-169: Velocity scaling applied
Eigen::VectorXd max_velocity_dummy_appended(velocity_limits.rows() + 1);
max_velocity_dummy_appended << (velocity_limits.col(1) * ci_profile->max_velocity_scaling_factor),
    std::numeric_limits<double>::max();

// Line 171-172: Acceleration scaling applied
Eigen::VectorXd max_acceleration_dummy_appended(acceleration_limits.rows() + 1);
max_acceleration_dummy_appended << (acceleration_limits.col(1) * ci_profile->max_acceleration_scaling_factor),
    std::numeric_limits<double>::max();

// Line 176: Scaled limits passed to TOTG algorithm
totg::Trajectory parameterized(path, max_velocity_dummy_appended, max_acceleration_dummy_appended, 0.001);
```

### How It Works Now

1. **Profile contains scaling factors** (lines 97-102):
   ```cpp
   ci_profile->max_velocity_scaling_factor     // Default: 1.0
   ci_profile->max_acceleration_scaling_factor // Default: 1.0
   ```

2. **Validation** (lines 97-102):
   ```cpp
   // Must be between 0.0 and 1.0
   if (!((ci_profile->max_velocity_scaling_factor > 0.0) &&
         (ci_profile->max_velocity_scaling_factor <= 1.0)))
     throw std::runtime_error("velocity scale factor must be greater than zero!");
   ```

3. **Application** (lines 168-171):
   ```cpp
   // Multiply limits by scaling factors BEFORE passing to TOTG
   velocity_limits.col(1) * ci_profile->max_velocity_scaling_factor
   acceleration_limits.col(1) * ci_profile->max_acceleration_scaling_factor
   ```

4. **Result:**
   - If `max_velocity_scaling_factor = 0.5`, velocity limits are halved
   - TOTG algorithm receives scaled limits
   - Trajectory respects the scaled limits

---

## Verification

### Test Case
```cpp
// Set scaling factors to 50%
profile->max_velocity_scaling_factor = 0.5;
profile->max_acceleration_scaling_factor = 0.5;

// Original limits: 1.0 m/s, 1.0 m/s²
// TOTG receives: 0.5 m/s, 0.5 m/s²
// Trajectory will be slower by 50%
```

### Expected Behavior
- ✅ Trajectory should take longer to execute (2x longer at 50% scaling)
- ✅ Maximum velocity should not exceed scaled limits
- ✅ Maximum acceleration should not exceed scaled limits

---

## Issue Status Assessment

### Was This Fixed?

**Evidence that it's FIXED:**
1. ✅ Scaling factors ARE multiplied with limits (lines 168, 171)
2. ✅ Scaled limits ARE passed to TOTG (line 176)
3. ✅ Validation ensures factors are in valid range (lines 97-102)

**Why Issue May Still Be Open:**
1. ⚠️ Fix was implemented but issue wasn't closed
2. ⚠️ No explicit PR or commit mentioned fixing this
3. ⚠️ Needs verification that fix works correctly
4. ⚠️ May need additional testing/documentation

### Git History

According to `git blame`, lines 168-172 were in the initial commit of the file (commit 4dcae88, September 6, 2025). This suggests:
- Either the fix was made sometime between 2021-2025
- Or the file was restructured and the history was reset

---

## Comparison with MoveIt2

### MoveIt2 Implementation

MoveIt2's TOTG uses scaling factors similarly:

```cpp
// MoveIt2 signature
bool computeTimeStamps(
    robot_trajectory::RobotTrajectory& trajectory,
    const double max_velocity_scaling_factor = 1.0,
    const double max_acceleration_scaling_factor = 1.0) const;

// Applied to limits before calling TOTG
Eigen::VectorXd max_velocity = joint_limits.velocity * max_velocity_scaling_factor;
Eigen::VectorXd max_acceleration = joint_limits.acceleration * max_acceleration_scaling_factor;
```

**Tesseract's implementation matches MoveIt2's approach** ✅

---

## Testing Recommendation

To verify this issue is truly fixed, should test:

### Test 1: Basic Scaling
```cpp
profile->max_velocity_scaling_factor = 0.5;
profile->max_acceleration_scaling_factor = 0.5;

// Compute trajectory
TOTG totg;
totg.compute(trajectory, env, profiles);

// Verify:
// 1. Trajectory duration ~2x longer than with scaling=1.0
// 2. Max velocity ≤ 0.5 * velocity_limits
// 3. Max acceleration ≤ 0.5 * acceleration_limits
```

### Test 2: Different Scalings
```cpp
// Test velocity scaling only
profile->max_velocity_scaling_factor = 0.25;
profile->max_acceleration_scaling_factor = 1.0;

// Verify trajectory is slower but can still accelerate fully
```

### Test 3: Edge Cases
```cpp
// Test minimum valid values
profile->max_velocity_scaling_factor = 0.001;
profile->max_acceleration_scaling_factor = 0.001;

// Test maximum valid values
profile->max_velocity_scaling_factor = 1.0;
profile->max_acceleration_scaling_factor = 1.0;

// Test invalid values should throw
profile->max_velocity_scaling_factor = 1.5;  // Should throw
profile->max_velocity_scaling_factor = 0.0;  // Should throw
```

---

## Related to Issue [#164](https://github.com/tesseract-robotics/tesseract_planning/issues/164)

**Connection:** Issue [#118](https://github.com/tesseract-robotics/tesseract_planning/issues/118) is about **trajectory-wide** scaling (which DOES work), while Issue [#164](https://github.com/tesseract-robotics/tesseract_planning/issues/164) is about **per-waypoint** scaling (which doesn't work by design).

| Feature | Issue | Status | Reason |
|---------|-------|--------|--------|
| **Uniform trajectory scaling** | [#118](https://github.com/tesseract-robotics/tesseract_planning/issues/118) | ✅ FIXED | Implemented at lines 168-171 |
| **Per-waypoint scaling** | [#164](https://github.com/tesseract-robotics/tesseract_planning/issues/164) | ⚠️ BY DESIGN | TOTG algorithm limitation |

---

## Recommendations

### For Tesseract Team

1. **✅ VERIFY FIX** - Run tests to confirm scaling factors work correctly
2. **✅ CLOSE ISSUE** - If tests pass, close issue [#118](https://github.com/tesseract-robotics/tesseract_planning/issues/118) as fixed
3. **✅ ADD TESTS** - Add unit tests for scaling factors to prevent regression
4. **✅ DOCUMENT** - Add examples of using scaling factors in documentation

### For Users

1. **✅ CAN USE** - Scaling factors should work correctly in current code
2. **⚠️ VERIFY** - Test your specific use case if relying on scaling
3. **⚠️ REPORT** - If scaling doesn't work, reopen or comment on issue [#118](https://github.com/tesseract-robotics/tesseract_planning/issues/118)

---

## Code Example: Using Scaling Factors

```cpp
#include <tesseract_time_parameterization/totg/time_optimal_trajectory_generation.h>
#include <tesseract_time_parameterization/totg/time_optimal_trajectory_generation_profiles.h>

// Create profile
auto profile = std::make_shared<TimeOptimalTrajectoryGenerationCompositeProfile>();

// Set scaling factors (0.0 < factor <= 1.0)
profile->max_velocity_scaling_factor = 0.5;      // 50% velocity
profile->max_acceleration_scaling_factor = 0.75;  // 75% acceleration

// Other parameters
profile->path_tolerance = 0.1;
profile->min_angle_change = 0.001;

// Add to profile dictionary
tesseract_common::ProfileDictionary profiles;
profiles.addProfile("TOTG", DEFAULT_PROFILE_KEY, profile);

// Compute trajectory with scaling
TimeOptimalTrajectoryGeneration totg("TOTG");
bool success = totg.compute(trajectory, env, profiles);

// Result: Trajectory will be slower (respecting 50% velocity limit)
```

---

## Summary

### Current Status
**Issue [#118](https://github.com/tesseract-robotics/tesseract_planning/issues/118) APPEARS TO BE FIXED** in the current codebase but the GitHub issue remains open.

### Evidence
- ✅ Scaling factors ARE being applied to limits (lines 168, 171)
- ✅ Scaled limits ARE passed to TOTG algorithm (line 176)
- ✅ Implementation matches MoveIt2's approach
- ✅ Validation ensures factors are in valid range

### Action Needed
1. **Test** - Verify scaling factors work correctly
2. **Close** - If tests pass, close issue [#118](https://github.com/tesseract-robotics/tesseract_planning/issues/118) on GitHub
3. **Document** - Add examples and tests

### Priority
🟢 **LOW** - Code appears correct, just needs verification and issue closure

---

## Comparison with Other Issues

| Issue | Type | Status | Fix Needed? |
|-------|------|--------|-------------|
| **[#27](https://github.com/tesseract-robotics/tesseract_planning/issues/27)** | Bug | Workaround | ✅ Yes (antiparallel) |
| **[#118](https://github.com/tesseract-robotics/tesseract_planning/issues/118)** | Bug | **APPEARS FIXED** | ⚠️ Verify & close |
| **[#164](https://github.com/tesseract-robotics/tesseract_planning/issues/164)** | Limitation | By design | ❌ No (document) |

**Issue [#118](https://github.com/tesseract-robotics/tesseract_planning/issues/118) stands out** as potentially already fixed but not officially closed.

---

## References

- [Tesseract Issue #118](https://github.com/tesseract-robotics/tesseract_planning/issues/118)
- [ISSUE_164_ANALYSIS.md](ISSUE_164_ANALYSIS.md) - Related per-waypoint scaling limitation
- [MoveIt2 TOTG Implementation](https://github.com/moveit/moveit2/blob/main/moveit_core/trajectory_processing/src/time_optimal_trajectory_generation.cpp)
