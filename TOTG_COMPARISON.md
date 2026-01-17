# Time-Optimal Trajectory Generation: MoveIt2 vs Tesseract Implementation Comparison

Generated: 2026-01-17

## Overview

Both implementations are based on the original Georgia Tech algorithm by Tobias Kunz (2012). This document highlights the key differences in implementation details, particularly focusing on numerical stability and robustness.

---

## 1. Epsilon Constants

### MoveIt2
```cpp
constexpr double EPS = 1e-6;  // Main epsilon
constexpr double angle_tolerance = 1e-05;
// Various hardcoded values: 0.000001, 0.999999
```

### Tesseract
```cpp
constexpr double EPS = 0.000001;  // (same as 1e-6)
// Uses tesseract_common::almostEqualRelativeAndAbs() for comparisons
```

**Analysis:** Both use the same epsilon value (1e-6), but Tesseract uses a more robust comparison function throughout.

---

## 2. CircularPathSegment Constructor

### MoveIt2 Implementation
```cpp
CircularPathSegment(const Eigen::VectorXd& start, const Eigen::VectorXd& intersection,
                    const Eigen::VectorXd& end, double max_deviation)
{
  // Check 1: Segment too short
  if ((intersection - start).norm() < 0.000001 || (end - intersection).norm() < 0.000001)
  {
    length_ = 0.0;
    radius_ = 1.0;
    center_ = intersection;
    x_ = Eigen::VectorXd::Zero(start.size());
    y_ = Eigen::VectorXd::Zero(start.size());
    return;
  }

  const Eigen::VectorXd start_direction = (intersection - start).normalized();
  const Eigen::VectorXd end_direction = (end - intersection).normalized();
  const double start_dot_end = start_direction.dot(end_direction);

  // Check 2: Parallel or antiparallel vectors (using dot product)
  if (start_dot_end > 0.999999 || start_dot_end < -0.999999)
  {
    length_ = 0.0;
    radius_ = 1.0;
    center_ = intersection;
    x_ = Eigen::VectorXd::Zero(start.size());
    y_ = Eigen::VectorXd::Zero(start.size());
    return;
  }

  const double angle = acos(start_dot_end);
  // ... rest of calculation
}
```

### Tesseract Implementation
```cpp
CircularPathSegment(const Eigen::VectorXd& start, const Eigen::VectorXd& intersection,
                    const Eigen::VectorXd& end, double max_deviation)
{
  // Check 1: Segment too short (identical to MoveIt2)
  if ((intersection - start).norm() < 0.000001 || (end - intersection).norm() < 0.000001)
  {
    length_ = 0.0;
    radius = 1.0;
    center = intersection;
    x = Eigen::VectorXd::Zero(start.size());
    y = Eigen::VectorXd::Zero(start.size());
    return;
  }

  const Eigen::VectorXd start_direction = (intersection - start).normalized();
  const Eigen::VectorXd end_direction = (end - intersection).normalized();

  // Check 2: Parallel vectors (using direction difference norm)
  if ((start_direction - end_direction).norm() < 0.000001)
  {
    length_ = 0.0;
    radius = 1.0;
    center = intersection;
    x = Eigen::VectorXd::Zero(start.size());
    y = Eigen::VectorXd::Zero(start.size());
    return;
  }

  // CRITICAL: Protect acos from NaN due to numerical errors
  // See https://github.com/ros-planning/moveit/pull/1861
  const double angle = acos(std::max(-1.0, start_direction.dot(end_direction)));
  // ... rest of calculation
}
```

### Key Differences

| Aspect | MoveIt2 | Tesseract |
|--------|---------|-----------|
| **Parallel check** | Dot product: `> 0.999999 \|\| < -0.999999` | Norm difference: `< 0.000001` |
| **acos protection** | None (relies on parallel check) | `std::max(-1.0, dot_product)` |
| **Antiparallel handling** | Explicitly checks both ends | Only checks parallel (norm difference) |

**IMPORTANT FINDING:**
- **MoveIt2** checks for both parallel (dot ≈ 1) and antiparallel (dot ≈ -1) cases
- **Tesseract** only checks for parallel cases using norm difference, which CANNOT detect antiparallel vectors
- **Tesseract** adds `std::max(-1.0, ...)` protection to prevent acos(x) with x < -1, which can occur with antiparallel vectors due to floating-point errors
- This suggests Tesseract may have a **potential bug** where antiparallel vectors could slip through and cause issues

---

## 3. integrateForward Function

### Key Section: Binary Search for Overshoot

Both implementations use nearly identical binary search logic:

```cpp
while (after - before > EPS)
{
  const double midpoint = 0.5 * (before + after);
  double midpoint_path_vel = 0.5 * (before_path_vel + after_path_vel);

  // ... convergence logic ...
}
```

### Critical Bug Found in MoveIt2

**MoveIt2 (line 759-761):**
```cpp
else
{
  if (getMinMaxPhaseSlope(trajectory.back().path_pos_, trajectory_.back().path_vel_, false) >
      getVelocityMaxPathVelocityDeriv(trajectory_.back().path_pos_))
  {
    return false;
  }
}
```

**Tesseract (line 758-764):**
```cpp
else
{
  if (getMinMaxPhaseSlope(trajectory.back().path_pos_, trajectory.back().path_vel_, false) >
      getVelocityMaxPathVelocityDeriv(trajectory_.back().path_pos_))
  {
    return false;
  }
}
```

**BUG:** MoveIt2 uses `trajectory_` (class member) instead of `trajectory` (function parameter). This could reference stale data and cause incorrect behavior!

---

## 4. integrateBackward Function

This is where the most significant differences exist.

### MoveIt2 Implementation
```cpp
void Trajectory::integrateBackward(std::list<TrajectoryStep>& start_trajectory,
                                   double path_pos, double path_vel, double acceleration)
{
  // ... setup code ...

  const double start_slope = (start2->path_vel_ - start1->path_vel_) /
                              (start2->path_pos_ - start1->path_pos_);
  const double intersection_path_pos =
      (start1->path_vel_ - path_vel + slope * path_pos - start_slope * start1->path_pos_) /
      (slope - start_slope);

  // INTERSECTION CHECK: Simple epsilon bounds
  if (std::max(start1->path_pos_, path_pos) - EPS <= intersection_path_pos &&
      intersection_path_pos <= EPS + std::min(start2->path_pos_, trajectory.front().path_pos_))
  {
    // ... merge trajectories ...
    return;
  }
}
```

### Tesseract Implementation
```cpp
void Trajectory::integrateBackward(std::list<TrajectoryStep>& start_trajectory,
                                   double path_pos, double path_vel, double acceleration)
{
  // ... setup code ...

  const double start_slope = (start2->path_vel_ - start1->path_vel_) /
                              (start2->path_pos_ - start1->path_pos_);

  // CRITICAL: Handle case where slopes are equal (avoids division by zero)
  bool check_eq_slope = tesseract_common::almostEqualRelativeAndAbs(slope, start_slope, EPS);
  double intersection_path_pos{ 0 };
  if (check_eq_slope)
    intersection_path_pos = start1->path_pos_ + (start2->path_pos_ - start1->path_pos_) / 2.0;
  else
    intersection_path_pos =
        (start1->path_vel_ - path_vel + slope * path_pos - start_slope * start1->path_pos_) /
        (slope - start_slope);

  // INTERSECTION CHECK: Robust comparison with almostEqual
  double pos_max = std::max(start1->path_pos_, path_pos);
  double pos_min = std::min(start2->path_pos_, trajectory.front().path_pos_);
  bool check1 = (pos_max < intersection_path_pos) ||
                tesseract_common::almostEqualRelativeAndAbs(pos_max, intersection_path_pos, EPS);
  bool check2 = (intersection_path_pos < pos_min) ||
                tesseract_common::almostEqualRelativeAndAbs(pos_min, intersection_path_pos, EPS);

  if (check1 && check2)
  {
    // ... merge trajectories ...
    return;
  }
}
```

### Key Differences

| Aspect | MoveIt2 | Tesseract |
|--------|---------|-----------|
| **Division by zero protection** | None | Checks if slopes are equal before dividing |
| **Intersection check** | Simple epsilon bounds | Robust `almostEqualRelativeAndAbs()` |
| **Equal slope handling** | Would result in NaN or Inf | Uses midpoint when slopes equal |
| **Comparison at line 780** | Simple `assert(start1->path_pos_ <= path_pos)` | Uses `almostEqualRelativeAndAbs()` |
| **Comparison at line 784** | Simple `if (start1->path_pos_ <= path_pos)` | Uses `almostEqualRelativeAndAbs()` |

---

## 5. Additional Numerical Stability Checks

### Tesseract-Specific Improvements

**Line 780:** Uses robust comparison in assertion
```cpp
assert(start1->path_pos_ < path_pos ||
       tesseract_common::almostEqualRelativeAndAbs(start1->path_pos_, path_pos, EPS));
```

**Line 784:** Uses robust comparison in condition
```cpp
if (start1->path_pos_ < path_pos ||
    tesseract_common::almostEqualRelativeAndAbs(start1->path_pos_, path_pos, EPS))
```

**Line 852:** Checks for zero before division
```cpp
if (!tesseract_common::almostEqualRelativeAndAbs(config_deriv[i], 0.0,
                                                   std::numeric_limits<double>::epsilon()))
```

---

## 6. Summary of Critical Issues

### MoveIt2 Issues Found

1. **Bug in integrateForward (line 759):** Uses `trajectory_` instead of `trajectory`
2. **No division-by-zero protection in integrateBackward:** When slopes are equal
3. **Less robust epsilon comparisons:** Uses simple arithmetic comparisons

### Tesseract Improvements

1. **Division-by-zero protection:** Explicitly handles equal slope case
2. **Robust comparisons:** Uses `almostEqualRelativeAndAbs()` throughout
3. **NaN protection in acos:** Uses `std::max(-1.0, dot_product)`
4. **Better numerical stability:** More careful handling of edge cases

### Tesseract Potential Issues

1. **Possible antiparallel bug in CircularPathSegment:** Only checks parallel case with norm difference, may not catch antiparallel vectors

---

## 7. Recommendations

### For Tesseract (Current Codebase)

1. **CircularPathSegment constructor:** Consider adding explicit antiparallel check:
   ```cpp
   const double start_dot_end = start_direction.dot(end_direction);
   if ((start_direction - end_direction).norm() < 0.000001 ||
       start_dot_end > 0.999999 || start_dot_end < -0.999999)
   ```

### For Understanding Differences

1. MoveIt2 has a **confirmed bug** at line 759 (using wrong variable)
2. Tesseract has **superior numerical stability** in integrateBackward
3. Both approaches to parallel detection have merit, but MoveIt2's is more comprehensive

---

## Files Referenced

- **MoveIt2 Source:** `/home/user/tesseract_planning/moveit2_totg_key_sections.cpp`
- **MoveIt2 Header:** `/home/user/tesseract_planning/moveit2_totg_header.hpp`
- **Tesseract Source:** `/home/user/tesseract_planning/tesseract_time_parameterization/totg/src/time_optimal_trajectory_generation.cpp`
- **Tesseract Header:** `/home/user/tesseract_planning/tesseract_time_parameterization/totg/include/tesseract_time_parameterization/totg/time_optimal_trajectory_generation.h`
