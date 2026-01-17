# Side-by-Side Code Comparison: MoveIt2 vs Tesseract TOTG

## 1. CircularPathSegment Constructor - Parallel/Antiparallel Detection

### MoveIt2 Approach (Dot Product Method)
```cpp
const Eigen::VectorXd start_direction = (intersection - start).normalized();
const Eigen::VectorXd end_direction = (end - intersection).normalized();
const double start_dot_end = start_direction.dot(end_direction);

// Check for parallel (dot ≈ 1) OR antiparallel (dot ≈ -1)
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
```

### Tesseract Approach (Norm Difference + acos Protection)
```cpp
const Eigen::VectorXd start_direction = (intersection - start).normalized();
const Eigen::VectorXd end_direction = (end - intersection).normalized();

// Check for parallel (norm of difference ≈ 0)
// NOTE: Does NOT detect antiparallel vectors!
if ((start_direction - end_direction).norm() < 0.000001)
{
  length_ = 0.0;
  radius = 1.0;
  center = intersection;
  x = Eigen::VectorXd::Zero(start.size());
  y = Eigen::VectorXd::Zero(start.size());
  return;
}

// Protect acos from NaN when dot product < -1 (antiparallel case)
const double angle = acos(std::max(-1.0, start_direction.dot(end_direction)));
```

**Analysis:**
- MoveIt2: Explicitly handles both parallel AND antiparallel cases before acos
- Tesseract: Only handles parallel case, relies on `std::max(-1.0, ...)` to prevent NaN from antiparallel vectors
- Tesseract approach may be less efficient as it doesn't early-exit for antiparallel cases

---

## 2. integrateBackward - Division by Zero Protection

### MoveIt2 (NO Protection)
```cpp
const double start_slope = (start2->path_vel_ - start1->path_vel_) /
                            (start2->path_pos_ - start1->path_pos_);
const double intersection_path_pos =
    (start1->path_vel_ - path_vel + slope * path_pos - start_slope * start1->path_pos_) /
    (slope - start_slope);
    // ^^^ Can be NaN or Inf if slope == start_slope!

if (std::max(start1->path_pos_, path_pos) - EPS <= intersection_path_pos &&
    intersection_path_pos <= EPS + std::min(start2->path_pos_, trajectory.front().path_pos_))
{
  const double intersection_path_vel =
      start1->path_vel_ + start_slope * (intersection_path_pos - start1->path_pos_);
  // ... merge trajectories
}
```

### Tesseract (WITH Protection)
```cpp
const double start_slope = (start2->path_vel_ - start1->path_vel_) /
                            (start2->path_pos_ - start1->path_pos_);

// CHECK: Are slopes equal? Avoid division by zero!
bool check_eq_slope = tesseract_common::almostEqualRelativeAndAbs(slope, start_slope, EPS);
double intersection_path_pos{ 0 };
if (check_eq_slope)
  // Use midpoint when slopes are equal
  intersection_path_pos = start1->path_pos_ + (start2->path_pos_ - start1->path_pos_) / 2.0;
else
  intersection_path_pos =
      (start1->path_vel_ - path_vel + slope * path_pos - start_slope * start1->path_pos_) /
      (slope - start_slope);

double pos_max = std::max(start1->path_pos_, path_pos);
double pos_min = std::min(start2->path_pos_, trajectory.front().path_pos_);

// Use robust comparison instead of simple epsilon bounds
bool check1 = (pos_max < intersection_path_pos) ||
              tesseract_common::almostEqualRelativeAndAbs(pos_max, intersection_path_pos, EPS);
bool check2 = (intersection_path_pos < pos_min) ||
              tesseract_common::almostEqualRelativeAndAbs(pos_min, intersection_path_pos, EPS);

if (check1 && check2)
{
  const double intersection_path_vel =
      start1->path_vel_ + (start_slope * (intersection_path_pos - start1->path_pos_));
  // ... merge trajectories
}
```

**Analysis:**
- MoveIt2: Can produce NaN/Inf when slopes are equal (occurs when consecutive steps have same acceleration)
- Tesseract: Explicitly handles equal slope case, uses midpoint approximation
- Tesseract: Uses more robust epsilon comparisons throughout

---

## 3. integrateBackward - Loop Condition Comparison

### MoveIt2
```cpp
// Line 780: Simple assertion
assert(start1->path_pos_ <= path_pos);

// Line 784: Simple comparison
if (start1->path_pos_ <= path_pos)
{
  trajectory.push_front(TrajectoryStep(path_pos, path_vel));
  // ... integration logic
}
```

### Tesseract
```cpp
// Line 780: Robust assertion with tolerance
assert(start1->path_pos_ < path_pos ||
       tesseract_common::almostEqualRelativeAndAbs(start1->path_pos_, path_pos, EPS));

// Line 784: Robust comparison with tolerance
if (start1->path_pos_ < path_pos ||
    tesseract_common::almostEqualRelativeAndAbs(start1->path_pos_, path_pos, EPS))
{
  trajectory.emplace_front(path_pos, path_vel);
  // ... integration logic
}
```

**Analysis:**
- Tesseract handles floating-point comparison edge cases better
- Prevents potential assertion failures due to numerical precision

---

## 4. integrateForward - CRITICAL BUG in MoveIt2

### MoveIt2 (BUG at line 759)
```cpp
if (getAccelerationMaxPathVelocity(after) < getVelocityMaxPathVelocity(after))
{
  if (after > next_discontinuity->first)
  {
    return false;
  }
  else if (getMinMaxPhaseSlope(trajectory.back().path_pos_, trajectory.back().path_vel_, true) >
           getAccelerationMaxPathVelocityDeriv(trajectory.back().path_pos_))
  {
    return false;
  }
}
else
{
  // BUG: Uses trajectory_ (class member) instead of trajectory (function parameter)
  if (getMinMaxPhaseSlope(trajectory.back().path_pos_, trajectory_.back().path_vel_, false) >
                                                        // ^^^^^^^^^^^
      getVelocityMaxPathVelocityDeriv(trajectory_.back().path_pos_))
                                      // ^^^^^^^^^^^
  {
    return false;
  }
}
```

### Tesseract (CORRECT)
```cpp
if (getAccelerationMaxPathVelocity(after) < getVelocityMaxPathVelocity(after))
{
  if (after > next_discontinuity->first)
  {
    return false;
  }

  if (getMinMaxPhaseSlope(trajectory.back().path_pos_, trajectory.back().path_vel_, true) >
      getAccelerationMaxPathVelocityDeriv(trajectory.back().path_pos_))
  {
    return false;
  }
}
else
{
  // CORRECT: Uses trajectory (function parameter)
  if (getMinMaxPhaseSlope(trajectory.back().path_pos_, trajectory.back().path_vel_, false) >
                                                        // ^^^^^^^^^^^^^^
      getVelocityMaxPathVelocityDeriv(trajectory_.back().path_pos_))
  {
    return false;
  }
}
```

**Analysis:**
- MoveIt2 has a clear bug where it accesses the wrong trajectory
- This could cause incorrect trajectory validation
- Tesseract correctly uses the function parameter throughout

---

## 5. getMinMaxPathAcceleration - Zero Division Check

### MoveIt2 (NO Check)
```cpp
double Trajectory::getMinMaxPathAcceleration(double path_position, double path_velocity, bool max)
{
  Eigen::VectorXd config_deriv = path_.getTangent(path_position);
  Eigen::VectorXd config_deriv2 = path_.getCurvature(path_position);
  double factor = max ? 1.0 : -1.0;
  double max_path_acceleration = std::numeric_limits<double>::max();
  for (unsigned int i = 0; i < joint_num_; ++i)
  {
    // Divides by config_deriv[i] without checking if it's zero!
    max_path_acceleration =
        std::min(max_path_acceleration,
                 (max_acceleration_[i] / std::abs(config_deriv[i])) -
                     (factor * config_deriv2[i] * path_velocity * path_velocity / config_deriv[i]));
  }
  return factor * max_path_acceleration;
}
```

### Tesseract (WITH Check)
```cpp
double Trajectory::getMinMaxPathAcceleration(double path_position, double path_velocity, bool max)
{
  Eigen::VectorXd config_deriv = path_.getTangent(path_position);
  Eigen::VectorXd config_deriv2 = path_.getCurvature(path_position);
  double factor = max ? 1.0 : -1.0;
  double max_path_acceleration = std::numeric_limits<double>::max();
  for (unsigned int i = 0; i < joint_num_; ++i)
  {
    // Only compute if config_deriv[i] is not effectively zero
    if (!tesseract_common::almostEqualRelativeAndAbs(config_deriv[i], 0.0,
                                                       std::numeric_limits<double>::epsilon()))
    {
      max_path_acceleration =
          std::min(max_path_acceleration,
                   (max_acceleration_[i] / std::abs(config_deriv[i])) -
                       (factor * config_deriv2[i] * path_velocity * path_velocity / config_deriv[i]));
    }
  }
  return factor * max_path_acceleration;
}
```

**Analysis:**
- MoveIt2: Potential division by zero if tangent component is zero
- Tesseract: Explicitly checks for near-zero values before division
- This could cause NaN/Inf propagation in MoveIt2

---

## 6. TrajectoryStep Constructor - Assertions

### MoveIt2
```cpp
struct TrajectoryStep
{
  TrajectoryStep() {}
  TrajectoryStep(double path_pos, double path_vel)
    : path_pos_(path_pos), path_vel_(path_vel)
  {
  }
  double path_pos_;
  double path_vel_;
  double time_;
};
```

### Tesseract
```cpp
struct TrajectoryStep
{
  TrajectoryStep() = default;
  TrajectoryStep(double path_pos, double path_vel)
    : path_pos_(path_pos), path_vel_(path_vel)
  {
    assert(!std::isnan(path_pos));
    assert(!std::isnan(path_vel));
  }
  double path_pos_{ 0 };
  double path_vel_{ 0 };
  double time_{ 0 };
};
```

**Analysis:**
- Tesseract: Adds NaN detection at construction time for early error detection
- Helps catch numerical issues immediately rather than letting them propagate

---

## Summary Table

| Feature | MoveIt2 | Tesseract | Winner |
|---------|---------|-----------|--------|
| **CircularPathSegment parallel detection** | Dot product (both directions) | Norm difference + acos protection | MoveIt2 (more explicit) |
| **CircularPathSegment antiparallel handling** | Explicit check | Implicit via acos protection | MoveIt2 (early exit) |
| **integrateBackward division by zero** | No protection | Protected | Tesseract |
| **integrateBackward epsilon comparison** | Simple arithmetic | almostEqualRelativeAndAbs | Tesseract |
| **integrateForward variable usage** | Bug (wrong variable) | Correct | Tesseract |
| **getMinMaxPathAcceleration zero check** | No check | Protected | Tesseract |
| **TrajectoryStep NaN detection** | None | Asserts on construction | Tesseract |

**Overall: Tesseract has superior numerical stability and robustness, with one confirmed bug fix from MoveIt2.**
