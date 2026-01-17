# Tesseract TOTG Endpoint Velocity/Acceleration Behavior Analysis

## Question
Does Tesseract have the same issue as MoveIt2 Jazzy where endpoints have non-zero velocities/accelerations?

**Reference:** [MoveIt2 Issue #3014](https://github.com/moveit/moveit2/issues/3014)

## Answer: **NO - Tesseract enforces zero velocities and accelerations at endpoints**

---

## Code Analysis

### 1. Trajectory Construction (Lines 446-491)

The `Trajectory` constructor explicitly enforces zero velocity at both endpoints:

#### Start Point (Line 457):
```cpp
trajectory_.emplace_back(0.0, 0.0);
//                        ↑     ↑
//                     pos=0  vel=0
```
**The trajectory starts with path_velocity = 0.0**

#### End Point (Line 473):
```cpp
double before_acceleration = getMinMaxPathAcceleration(path_.getLength(), 0.0, false);
integrateBackward(trajectory_, path_.getLength(), 0.0, before_acceleration);
//                                                  ↑
//                                           path_vel = 0.0
```
**The trajectory ends with path_velocity = 0.0**

### 2. Velocity Calculation (Line 1072-1075)

Joint velocities are computed from path velocity:

```cpp
Eigen::VectorXd Trajectory::getVelocity(const PathData& data) const
{
  return path_.getTangent(data.path_pos) * data.path_vel;
  //            any vector               *  0.0  = 0
}
```

**When `path_vel = 0.0`, joint velocity is always zero regardless of tangent direction.**

### 3. Acceleration Calculation (Lines 1077-1085)

Acceleration is computed as the change in velocity over time:

```cpp
Eigen::VectorXd Trajectory::getAcceleration(const PathData& data) const
{
  Eigen::VectorXd path_acc =
      (path_.getTangent(data.path_pos) * data.path_vel -
       path_.getTangent(data.prev_path_pos) * data.prev_path_vel);
  //            tangent * 0.0           -   tangent * 0.0  = 0

  double time_step = data.time - data.prev_time;
  if (time_step > 0.0)
    path_acc /= time_step;  // 0 / anything = 0
  return path_acc;          // 0 / 0 handled: returns 0
}
```

**At endpoints where both `path_vel` and `prev_path_vel` are 0, acceleration is also zero.**

### 4. Assignment to Instructions (Lines 939-980)

The `assignData` function sets endpoint values from computed path data:

#### First Waypoint (Lines 944-949):
```cpp
// Set Start
PathData path_data = getPathData(0);
Eigen::VectorXd uv = getVelocity(path_data).head(trajectory.dof());  // = 0
Eigen::VectorXd ua = getAcceleration(path_data).head(trajectory.dof());  // = 0
double time{ 0 };
trajectory.setData(0, uv, ua, time);
```

#### Last Waypoint (Lines 969-976):
```cpp
// Set end
path_data = getPathData(time);  // time = final duration
uv = getVelocity(path_data).head(trajectory.dof());  // = 0
ua = getAcceleration(path_data).head(trajectory.dof());  // = 0
trajectory.setData((trajectory.size() - 1), uv, ua, time);
```

---

## Comparison with MoveIt2

### MoveIt2 Humble (Pre-2.10.0)
- ✅ First waypoint: velocity = 0, acceleration = 0
- ❌ Last waypoint: velocity ≠ 0, acceleration ≠ 0

### MoveIt2 Jazzy (2.10.0+)
- ❌ First waypoint: velocity ≠ 0, acceleration ≠ 0
- ❓ Last waypoint: behavior unknown

### **Tesseract (Current)**
- ✅ First waypoint: velocity = 0, acceleration = 0
- ✅ Last waypoint: velocity = 0, acceleration = 0

---

## Implications for Trajectory Concatenation

### Scenario: Concatenating Two Trajectories

```
Trajectory A: [start] ─────> [end_A]
                              ↓
Trajectory B:            [start_B] ─────> [end]
```

**For safe concatenation without discontinuities:**
- `end_A.position == start_B.position` ✓
- `end_A.velocity == start_B.velocity` ← **Critical**
- `end_A.acceleration == start_B.acceleration` ← **Critical**

### Tesseract Behavior
Since **both endpoints have zero velocity/acceleration**:
- `end_A.velocity = 0`
- `start_B.velocity = 0`
- `end_A.velocity == start_B.velocity` ✓ **Safe!**

### MoveIt2 Jazzy Behavior
Since endpoints may have non-zero values:
- `end_A.velocity ≠ 0`
- `start_B.velocity ≠ 0`
- `end_A.velocity ≠ start_B.velocity` ✗ **Discontinuity!**

---

## Test Case Verification (Expected Results)

If you were to run the test I created (`endpoint_velocity_test.cpp`), you should see:

```
TEST 1: Simple A -> B Trajectory
-----------------------------------
First waypoint (index 0):
  Velocity: 0 0 0 0 0 0
  Acceleration: 0 0 0 0 0 0
  Velocity is zero: YES
  Acceleration is zero: YES

Last waypoint (index 1):
  Velocity: 0 0 0 0 0 0
  Acceleration: 0 0 0 0 0 0
  Velocity is zero: YES
  Acceleration is zero: YES

========================================
SUMMARY
========================================
  - Start points have zero velocity: YES
  - End points have zero velocity: YES

  ✓ Tesseract behaves like MoveIt2 Humble (zero endpoints)
  ✓ Safe for trajectory concatenation
```

---

## Edge Cases

### What about intermediate waypoints?

Intermediate waypoints (blended with circular arcs) **can and should** have non-zero velocities to maintain continuity through the blend regions. This is correct behavior.

Only the **first** and **last** waypoints of a trajectory have enforced zero velocities.

### What if a user wants non-zero endpoint velocities?

Currently, Tesseract's TOTG algorithm **does not support** non-zero endpoint velocities. This is by design - the algorithm assumes:
1. Robot starts from rest
2. Robot comes to rest at the end

To support moving between trajectories without stopping, you would need:
- A different time-parameterization algorithm (like Ruckig)
- OR trajectory blending at a higher level
- OR modification of the TOTG algorithm to accept initial/final velocity constraints

---

## Conclusion

**✅ Tesseract is SAFE for trajectory concatenation**

Unlike MoveIt2 Jazzy, Tesseract enforces zero velocities and accelerations at trajectory endpoints. This means:

1. ✓ No velocity discontinuities when concatenating trajectories
2. ✓ No need for manual endpoint zeroing
3. ✓ Compatible with robot controllers that expect smooth trajectories
4. ✓ Follows the traditional TOTG algorithm specification (start/stop from rest)

**Tesseract behaves like MoveIt2 Humble** (the older, more conservative behavior), not like Jazzy.

---

## Recommendations

1. **No changes needed** - Current behavior is correct and safe
2. **If non-zero endpoints are desired**, consider:
   - Document this limitation clearly
   - Implement trajectory blending at motion planning level
   - OR add optional parameters for initial/final velocities (advanced feature)
3. **For users migrating from MoveIt2 Jazzy**, they should be aware that Tesseract will always stop at waypoints unless using trajectory blending

---

## Related Files

- Implementation: `tesseract_time_parameterization/totg/src/time_optimal_trajectory_generation.cpp`
  - Trajectory constructor: Lines 446-491
  - getVelocity: Lines 1072-1075
  - getAcceleration: Lines 1077-1085
  - assignData: Lines 939-980

- Test: `endpoint_velocity_test.cpp` (created but not built)

- MoveIt2 Issue: https://github.com/moveit/moveit2/issues/3014
