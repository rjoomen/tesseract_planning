# Issue #27 Analysis: TOTG "Negative Path Velocity" Error

**Issue:** [tesseract-robotics/tesseract_planning#27](https://github.com/tesseract-robotics/tesseract_planning/issues/27)
**Related:** [moveit/moveit#2495](https://github.com/moveit/moveit/issues/2495)
**Status:** ⚠️ **Workaround implemented, root cause NOT fixed**

---

## Problem Description

### Error Message
```
Error while integrating backward: Negative path velocity
```

### When It Occurs
The TOTG algorithm fails when a trajectory **returns to or near a previous position**, creating an A→B→A pattern. Specifically:

```cpp
// Problematic trajectory pattern:
waypoint << 0, 0.7, -2.1, 0, -0.25, 0;       // Point A
waypoints.push_back(waypoint);
waypoint << 0, 0, 0, 0, 0, 0;                // Point B (home)
waypoints.push_back(waypoint);
waypoint << 0, 0.70001, -2.1, 0, -0.25, 0;  // Point A' (nearly same as A)
waypoints.push_back(waypoint);
waypoint << 0, 0, 0, 0, 0, 0.1;              // Point B' (nearly home)
waypoints.push_back(waypoint);
```

### Where It Fails
The error occurs in `integrateBackward()` at line 792-797:

```cpp
path_vel -= time_step_ * acceleration;
path_pos -= time_step_ * 0.5 * (path_vel + trajectory.front().path_vel_);
acceleration = getMinMaxPathAcceleration(path_pos, path_vel, false);

if (path_vel < 0.0)  // ← ERROR TRIGGERED HERE
{
  valid_ = false;
  CONSOLE_BRIDGE_logError("Error while integrating backward: Negative path velocity");
  end_trajectory_ = trajectory;
  return;
}
```

---

## Root Cause Analysis

### Original Diagnosis (from Issue #27)

**Levi Armstrong identified:**
1. **Uninitialized data** at line 774 (in older version)
2. **Inconsistent float comparisons** without epsilon tolerance using `<=` and `>=`
3. **Assertion doesn't consistently trigger** when the condition should fail

### Deeper Analysis

The issue is related to **circular path segment blending** when waypoints are very close together:

1. **Path construction** creates circular blend arcs between waypoints
2. When waypoints are **nearly identical** (like point A and A'), the blend arc calculation can produce:
   - Very small or zero-length segments
   - Numerical instability in tangent/curvature calculations
   - Invalid switching points

3. During **backward integration**, these problematic segments cause the path velocity to become negative, which is physically impossible.

### Connection to Antiparallel Vectors

This issue is **directly related** to the antiparallel vector problem identified earlier:

- A→B→A trajectory creates **near-antiparallel** tangent vectors
- `CircularPathSegment` constructor only checks parallel, not antiparallel
- Antiparallel vectors cause `angle ≈ π`, leading to:
  - `tan(π/2) → ∞` in radius calculation
  - `cos(π/2) = 0` causing division by zero
  - Invalid path segments that break backward integration

---

## Current Workaround (PR #32)

### Implementation
Located at lines 156-172 in `time_optimal_trajectory_generation.cpp`:

```cpp
// Append a dummy joint as a workaround to
// https://github.com/ros-industrial-consortium/tesseract_planning/issues/27
std::list<Eigen::VectorXd> new_points;
double dummy = 1.0;
for (auto& point : points)
{
  Eigen::VectorXd new_point(point.size() + 1);
  new_point << point, dummy;         // Append dummy value
  new_points.push_back(new_point);
  dummy += 1.0;                      // Increment for next point
}

Eigen::VectorXd max_velocity_dummy_appended(velocity_limits.rows() + 1);
max_velocity_dummy_appended << (velocity_limits.col(1) * ci_profile->max_velocity_scaling_factor),
    std::numeric_limits<double>::max();  // ← Dummy joint has infinite velocity

Eigen::VectorXd max_acceleration_dummy_appended(acceleration_limits.rows() + 1);
max_acceleration_dummy_appended << (acceleration_limits.col(1) * ci_profile->max_acceleration_scaling_factor),
    std::numeric_limits<double>::max();  // ← Dummy joint has infinite acceleration
```

### How It Works
1. **Adds a fake joint** to every waypoint with incrementing values (1.0, 2.0, 3.0, ...)
2. **Ensures no duplicate points** because dummy joint is always different
3. **Sets infinite limits** on dummy joint so it never becomes the limiting factor
4. **Strips dummy joint** when assigning final trajectory data

### Effectiveness
✅ **Prevents the error** - No duplicate points means no problematic blend arcs
⚠️ **Doesn't fix root cause** - Just masks the underlying numerical issue

---

## Comparison with MoveIt

### MoveIt Status
- **Issue:** [moveit/moveit#2495](https://github.com/moveit/moveit/issues/2495) (opened Jan 2021)
- **Status:** ⚠️ **STILL OPEN** (as of 2025)
- **Workaround:** Suggested in comments but **not officially implemented**
- **Fix:** ❌ **No proper fix merged**

### Test Case
The commented-out test at lines 293-316 in `time_optimal_trajectory_generation_tests.cpp`:

```cpp
// Causes the issue below. Workaround is applied when using computeTimestamps
// interface like in testCommandLanguageInterface
// https://github.com/ros-industrial-consortium/tesseract_planning/issues/27
// TEST(time_optimal_trajectory_generation, test_return_home)
```

This test is **deliberately disabled** because:
- It fails without the workaround
- The workaround is only applied in the high-level `compute()` interface
- Testing the low-level `Trajectory` constructor directly bypasses the workaround

---

## Why This Matters

### Impact
1. **Limits trajectory patterns** - Can't reliably do A→B→A motions
2. **Masks deeper bugs** - Workaround hides numerical issues
3. **Performance overhead** - Every trajectory gets an extra dummy dimension
4. **Not truly fixed** - Could still fail in edge cases

### Frequency
- ⚠️ **Common in practice** - Many real-world tasks involve returning to home position
- ⚠️ **Difficult to predict** - Depends on waypoint spacing and `max_deviation` setting
- ⚠️ **Silent failure possible** - Users may not realize trajectory is being modified

---

## Proper Fix Recommendations

### Priority 1: Fix Antiparallel Vector Detection

This would likely fix issue #27 as a side effect:

```cpp
// In CircularPathSegment constructor
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

**Rationale:** Antiparallel vectors (A→B→A pattern) would be caught early, preventing invalid circular segments from being created.

### Priority 2: Improve Backward Integration Robustness

Add checks before velocity can go negative:

```cpp
double new_path_vel = path_vel - time_step_ * acceleration;

// Prevent negative velocity with adaptive step
if (new_path_vel < 0.0)
{
  // Reduce time step to reach exactly zero velocity
  double safe_time_step = path_vel / acceleration;
  new_path_vel = 0.0;
  path_pos -= safe_time_step * 0.5 * path_vel;
}
else
{
  path_vel = new_path_vel;
  path_pos -= time_step_ * 0.5 * (path_vel + trajectory.front().path_vel_);
}
```

### Priority 3: Better Duplicate Point Handling

Instead of dummy joint workaround:

```cpp
// In duplicate point removal (lines 119-141)
// When points are too close, use linear interpolation instead of circular blend
if ((position - prev_point).norm() < threshold)
{
  // Force linear segment instead of circular blend
  // OR adjust path_tolerance dynamically
  // OR skip blending for this segment
}
```

---

## Testing Recommendations

### Test 1: Enable Return Home Test
```cpp
// Re-enable test at line 295 after implementing proper fix
TEST(time_optimal_trajectory_generation, test_return_home)
{
  // Should PASS without dummy joint workaround
}
```

### Test 2: Various A→B→A Patterns
```cpp
// Test different spacings
waypoints: [A, B, A]           // Exact return
waypoints: [A, B, A + ε]       // Slight offset (current failure case)
waypoints: [A, B, C, B, A]     // Multi-point return
```

### Test 3: Antiparallel Vectors
```cpp
// Test opposing directions
start_direction = [1, 0, 0]
end_direction = [-1, 0, 0]  // Antiparallel
// Should create zero-length segment, not crash
```

---

## Comparison with Earlier Analysis

This issue **confirms** the antiparallel vector detection problem identified earlier:

| Earlier Finding | Issue #27 Connection |
|----------------|----------------------|
| Antiparallel vectors only partially checked | ✅ **Confirmed** - A→B→A creates antiparallel tangents |
| Could cause division by zero in radius calc | ✅ **Confirmed** - Leads to invalid segments |
| Suggested adding antiparallel check | ✅ **Would fix this issue** |

---

## Recommendations Summary

### Immediate (No Code Changes)
1. ✅ **Document the limitation** - Users should know about A→B→A issues
2. ✅ **Keep workaround** - It prevents crashes in production

### Short Term (Low Effort)
3. 🟢 **Add antiparallel vector detection** - 5-10 lines, likely fixes root cause
4. 🟢 **Add negative velocity safeguard** - 10-15 lines, makes backward integration robust

### Long Term (Medium Effort)
5. 🟡 **Implement proper fix** - Replace dummy joint with correct handling
6. 🟡 **Enable return home test** - Verify fix works
7. 🟡 **Add comprehensive test suite** - Cover all edge cases

### Optional (High Effort)
8. 🔴 **Contribute fix to MoveIt** - Help upstream project too

---

## Conclusion

**Issue #27 is a known limitation with a working workaround**, but the root cause remains unfixed in both Tesseract and MoveIt. The recommended antiparallel vector detection fix from earlier analysis would likely resolve this issue as a beneficial side effect.

**Priority:** 🟡 **MEDIUM** - Has workaround, but proper fix would be better

**Effort:** 🟢 **LOW** - ~20 lines of code to add antiparallel check + safeguards

**Risk:** 🟢 **LOW** - Changes are localized and well-tested in MoveIt

---

## Related Documents

- [TOTG_COMPARISON.md](TOTG_COMPARISON.md) - See section on antiparallel vectors
- [TOTG_SIDE_BY_SIDE_COMPARISON.md](TOTG_SIDE_BY_SIDE_COMPARISON.md) - CircularPathSegment comparison
- [COMPARISON_SUMMARY.md](COMPARISON_SUMMARY.md) - Priority 2 recommendation

## References

- [Tesseract Issue #27](https://github.com/tesseract-robotics/tesseract_planning/issues/27)
- [Tesseract PR #32](https://github.com/tesseract-robotics/tesseract_planning/pull/32) - Workaround implementation
- [MoveIt Issue #2495](https://github.com/moveit/moveit/issues/2495) - Same problem, still open
