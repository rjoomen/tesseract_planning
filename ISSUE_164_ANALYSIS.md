# Issue #164 Analysis: TOTG State Scaling Limitation

**Issue:** [tesseract-robotics/tesseract_planning#164](https://github.com/tesseract-robotics/tesseract_planning/issues/164)
**Status:** ⚠️ **OPEN - Known Limitation** (since January 19, 2022)
**Severity:** 🟡 **MEDIUM** - Limits flexibility but doesn't cause crashes

---

## Problem Description

### Issue Summary
> "An attempt to support this it was implemented to post scale the individual states which actually causes undesirable trajectories"

**What TOTG supports:**
- ✅ **Uniform trajectory scaling** - Apply same velocity/acceleration scaling to entire trajectory

**What TOTG does NOT support:**
- ❌ **Independent state scaling** - Different velocity/acceleration scaling per waypoint/state
- ❌ **Per-waypoint velocity limits** - Slow down at specific points while going fast elsewhere

### The Problem
The TOTG algorithm is fundamentally designed to find **time-optimal** trajectories that keep velocity **as high as possible** while respecting global limits. When you try to artificially slow down individual waypoints after the fact (post-scaling), you create:

1. **Discontinuous trajectories** - Velocity/acceleration jumps at waypoint boundaries
2. **Limit violations** - Actual acceleration can exceed specified limits
3. **Non-optimal paths** - Trajectory deviates from the mathematically optimal solution

---

## Technical Background

### How TOTG Works

TOTG operates in a **1D phase plane** where:
- **s** = path position (0 to path_length)
- **ṡ** = path velocity (derivative of s w.r.t. time)
- **s̈** = path acceleration

The algorithm finds the **maximum bang-bang control**:
```
s̈ = +a_max  (accelerate as hard as possible)
s̈ = -a_max  (decelerate as hard as possible)
s̈ = 0       (coast at maximum velocity)
```

This creates a **globally time-optimal trajectory** that cannot be arbitrarily modified at individual states without breaking optimality.

### Current Implementation (Uniform Scaling)

In `time_optimal_trajectory_generation.cpp` lines 168-171:

```cpp
Eigen::VectorXd max_velocity_dummy_appended(velocity_limits.rows() + 1);
max_velocity_dummy_appended << (velocity_limits.col(1) * ci_profile->max_velocity_scaling_factor),
    std::numeric_limits<double>::max();

Eigen::VectorXd max_acceleration_dummy_appended(acceleration_limits.rows() + 1);
max_acceleration_dummy_appended << (acceleration_limits.col(1) * ci_profile->max_acceleration_scaling_factor),
    std::numeric_limits<double>::max();
```

**Key point:** Single scaling factors (`max_velocity_scaling_factor`, `max_acceleration_scaling_factor`) applied to ALL joints for the ENTIRE trajectory.

---

## Why Per-State Scaling is Problematic

### Example Scenario
```cpp
// Desired: Slow down at waypoint 2, fast elsewhere
waypoint[0]: position = A, desired_speed = 100%
waypoint[1]: position = B, desired_speed = 100%
waypoint[2]: position = C, desired_speed = 25%   // ← Want to slow down here
waypoint[3]: position = D, desired_speed = 100%
```

### Failed Approach: Post-Scaling
```cpp
// TOTG computes optimal trajectory
trajectory = TOTG(waypoints, max_vel, max_accel)

// Then artificially scale state 2
trajectory[2].velocity *= 0.25      // ← WRONG!
trajectory[2].acceleration *= 0.25   // ← WRONG!
```

**Problems:**
1. **Discontinuity** at waypoint 2:
   - Before: v = 1.0 m/s
   - At: v = 0.25 m/s (scaled)
   - After: v = 1.0 m/s
   - **Result:** Instantaneous velocity change = impossible!

2. **Acceleration violation** during transition:
   - To go from 1.0 → 0.25 m/s requires deceleration
   - To go from 0.25 → 1.0 m/s requires acceleration
   - These transitions weren't part of TOTG calculation
   - **Result:** May exceed `max_acceleration` limits!

3. **Path deviation**:
   - TOTG carefully planned the path position for each time step
   - Changing velocities changes where you are at each time
   - **Result:** Robot may deviate from intended path!

---

## Comparison with Other Approaches

### 1. Ruckig (Jerk-Limited Smoothing)

**Status in Tesseract:** ✅ Has Ruckig implementation

**Capabilities:**
- ✅ Supports **non-zero initial/final velocities**
- ✅ Supports **jerk limits**
- ⚠️ **Still doesn't support per-waypoint scaling**

Ruckig has the same fundamental limitation - it computes globally optimal trajectories.

### 2. Iterative Spline Parameterization (ISP)

**Status in Tesseract:** ✅ Has ISP implementation

**Capabilities:**
- ✅ May be **more flexible** for per-waypoint constraints
- ⚠️ **Not time-optimal** (trades optimality for flexibility)

ISP might support per-waypoint velocity constraints better than TOTG.

### 3. MoveIt Approach

**MoveIt TOTG:** Same limitation as Tesseract

MoveIt's TOTG also only supports uniform trajectory-wide scaling factors:
```cpp
bool computeTimeStamps(robot_trajectory::RobotTrajectory& trajectory,
                      const double max_velocity_scaling_factor = 1.0,
                      const double max_acceleration_scaling_factor = 1.0)
```

Single scaling factors for entire trajectory.

---

## Use Cases Affected

### ❌ Cannot Do (with TOTG)

1. **Slow down at dangerous points:**
   ```
   A ──fast──> B ──slow──> C ──fast──> D
                    ↑ near obstacle
   ```

2. **Variable speed for process constraints:**
   ```
   weld_start ──slow─> weld_end ──fast──> next_point
                ↑ welding requires slow speed
   ```

3. **Speed-dependent tool activation:**
   ```
   approach ──fast──> activate_tool ──slow──> work ──fast──> retract
                            ↑ need to slow down to activate
   ```

### ✅ Can Do (Workarounds)

1. **Split into multiple trajectories:**
   ```cpp
   // Trajectory 1: A → B (fast, scaling = 1.0)
   traj1 = TOTG(waypoints_AB, scaling=1.0)

   // Trajectory 2: B → C (slow, scaling = 0.25)
   traj2 = TOTG(waypoints_BC, scaling=0.25)

   // Trajectory 3: C → D (fast, scaling = 1.0)
   traj3 = TOTG(waypoints_CD, scaling=1.0)
   ```

   **Pros:** Achieves desired speed variation
   **Cons:** More complex, endpoint velocity = 0 (Tesseract enforces this)

2. **Use different time parameterization algorithm:**
   ```cpp
   // Use ISP instead of TOTG for trajectories needing per-waypoint control
   IterativeSplineParameterization isp;
   // May support per-waypoint constraints better
   ```

3. **Adjust path planning:**
   ```cpp
   // Add more dense waypoints in slow regions
   // Fewer waypoints in fast regions
   // TOTG will naturally take longer through dense regions
   ```

---

## Levi Armstrong's Recommendation

From Issue #164 comments:

> "Disable independent state scaling functionality for this algorithm"

**Rationale:**
- Post-scaling creates **incorrect** trajectories
- Better to **not support** the feature than to support it incorrectly
- Users should use alternative approaches instead

---

## Comparison with Earlier Findings

This issue is **independent** of the other issues found:

| Issue | Cause | Impact | Status |
|-------|-------|--------|--------|
| **#27 - Negative velocity** | Antiparallel vectors | Crashes on A→B→A | Has workaround |
| **#164 - State scaling** | Algorithm limitation | Cannot vary speed per waypoint | Fundamental limit |
| **MoveIt #2495** | Same as Tesseract #27 | Same issue | Unfixed |
| **Antiparallel detection** | Missing check | Potential division by zero | Easy fix |

**Issue #164 is a fundamental algorithmic limitation, not a bug.**

---

## Recommendations

### For Tesseract Development

1. **✅ Document the limitation clearly**
   - Add to TOTG profile documentation
   - Explain that only uniform scaling is supported
   - Provide workaround examples

2. **⚠️ Consider removing misleading features**
   - If there's any UI/API that suggests per-state scaling works
   - Better to remove it than have it create bad trajectories

3. **⚠️ Provide alternative solutions**
   - Document when to use ISP vs TOTG vs Ruckig
   - Provide examples of trajectory splitting approach

### For Users

1. **Use trajectory splitting** for variable speed requirements
2. **Use ISP** if per-waypoint control is more important than time-optimality
3. **Adjust waypoint density** to naturally create speed variations
4. **Accept uniform scaling** as TOTG's intended design

---

## Relation to MoveIt2 Comparison

### Does MoveIt2 support per-state scaling?

❌ **NO** - MoveIt2 has the same limitation

**Evidence:**
```cpp
// MoveIt2 API (from earlier analysis)
bool computeTimeStamps(
    robot_trajectory::RobotTrajectory& trajectory,
    const double max_velocity_scaling_factor = 1.0,      // ← Single value
    const double max_acceleration_scaling_factor = 1.0)  // ← Single value
```

**Conclusion:** This is not a missing feature in Tesseract - it's a fundamental property of the TOTG algorithm itself.

---

## Impact Assessment

### Severity
🟡 **MEDIUM** - Limits flexibility but doesn't prevent TOTG from working

### Frequency
🟢 **LOW** - Most use cases work fine with uniform scaling

### Workarounds Available
✅ **YES** - Multiple viable alternatives exist

### Should This Be "Fixed"?
❌ **NO** - This is a fundamental algorithmic property, not a bug

**Better approach:** Document limitations and provide alternative solutions

---

## Summary

**Issue #164 describes a fundamental limitation of the TOTG algorithm**, not a bug that needs fixing. The algorithm is designed to compute globally time-optimal trajectories with uniform limits. Attempting to apply per-waypoint scaling after the fact creates invalid trajectories.

**Key Takeaways:**
1. ✅ TOTG works correctly for its intended use case (uniform scaling)
2. ❌ TOTG cannot support per-waypoint scaling without being a different algorithm
3. ✅ Workarounds exist (trajectory splitting, different algorithms)
4. ✅ MoveIt2 has the same limitation - this is expected behavior

**Priority:** 🟢 **LOW** - Document limitation, no code changes needed

**Action Items:**
- Add documentation explaining the limitation
- Provide examples of workarounds
- Clarify when to use TOTG vs ISP vs Ruckig

---

## References

- [Tesseract Issue #164](https://github.com/tesseract-robotics/tesseract_planning/issues/164)
- [TOTG Algorithm Paper](https://www.researchgate.net/publication/265876699_Time-Optimal_Trajectory_Generation_for_Path_Following_with_Bounded_Acceleration_and_Velocity) - Kunz & Stilman
- [Modern Robotics - Time-Optimal Time Scaling](https://modernrobotics.northwestern.edu/nu-gm-book-resource/9-4-time-optimal-time-scaling-part-2-of-3/)
- [PickNik TOTG Workshop](https://picknik.ai/docs/moveit_workshop_macau_2019/TOTG.pdf)
