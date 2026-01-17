/*
 * MoveIt2 Time-Optimal Trajectory Generation - Key Sections
 * Extracted from: https://raw.githubusercontent.com/moveit/moveit2/main/moveit_core/trajectory_processing/src/time_optimal_trajectory_generation.cpp
 * Date: 2026-01-17
 *
 * Key sections for comparison with Tesseract implementation
 */

// ============================================================================
// NUMERICAL STABILITY CONSTANTS
// ============================================================================

constexpr double EPS = 1e-6;  // Main epsilon for numerical comparisons
constexpr double angle_tolerance = 1e-05;  // Angle tolerance

// ============================================================================
// CIRCULARPATHSEGMENT CONSTRUCTOR (lines ~95-145)
// ============================================================================

CircularPathSegment(const Eigen::VectorXd& start, const Eigen::VectorXd& intersection, const Eigen::VectorXd& end,
                    double max_deviation)
{
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

  // CRITICAL: Numerical stability check for parallel/antiparallel vectors
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
  const double start_distance = (start - intersection).norm();
  const double end_distance = (end - intersection).norm();

  double distance = std::min(start_distance, end_distance);
  distance = std::min(distance, max_deviation * sin(0.5 * angle) / (1.0 - cos(0.5 * angle)));

  radius_ = distance / tan(0.5 * angle);
  length_ = angle * radius_;

  center_ = intersection + (end_direction - start_direction).normalized() * radius_ / cos(0.5 * angle);
  x_ = (intersection - distance * start_direction - center_).normalized();
  y_ = start_direction;
}

// ============================================================================
// INTEGRATEFORWARD FUNCTION (lines ~850-950)
// ============================================================================

bool Trajectory::integrateForward(std::list<TrajectoryStep>& trajectory, double acceleration)
{
  double path_pos = trajectory.back().path_pos_;
  double path_vel = trajectory.back().path_vel_;

  std::list<std::pair<double, bool>> switching_points = path_.getSwitchingPoints();
  std::list<std::pair<double, bool>>::iterator next_discontinuity = switching_points.begin();

  while (true)
  {
    while ((next_discontinuity != switching_points.end()) &&
           (next_discontinuity->first <= path_pos || !next_discontinuity->second))
    {
      ++next_discontinuity;
    }

    double old_path_pos = path_pos;
    double old_path_vel = path_vel;

    path_vel += time_step_ * acceleration;
    path_pos += time_step_ * 0.5 * (old_path_vel + path_vel);

    // CRITICAL: Numerical stability check when approaching discontinuity
    if (next_discontinuity != switching_points.end() && path_pos > next_discontinuity->first)
    {
      if (path_pos - next_discontinuity->first < EPS)
      {
        continue;
      }
      path_vel = old_path_vel +
                 (next_discontinuity->first - old_path_pos) * (path_vel - old_path_vel) / (path_pos - old_path_pos);
      path_pos = next_discontinuity->first;
    }

    if (path_pos > path_.getLength())
    {
      trajectory.push_back(TrajectoryStep(path_pos, path_vel));
      return true;
    }
    else if (path_vel < 0.0)
    {
      valid_ = false;
      RCLCPP_ERROR(getLogger(), "Error while integrating forward: Negative path velocity");
      return true;
    }

    if (path_vel > getVelocityMaxPathVelocity(path_pos) &&
        getMinMaxPhaseSlope(old_path_pos, getVelocityMaxPathVelocity(old_path_pos), false) <=
            getVelocityMaxPathVelocityDeriv(old_path_pos))
    {
      path_vel = getVelocityMaxPathVelocity(path_pos);
    }

    trajectory.push_back(TrajectoryStep(path_pos, path_vel));
    acceleration = getMinMaxPathAcceleration(path_pos, path_vel, true);

    if (path_vel == 0 && acceleration == 0)
    {
      valid_ = false;
      RCLCPP_ERROR(getLogger(), "Error while integrating forward: zero acceleration and velocity. Are any relevant "
                                "acceleration components limited to zero?");
      return true;
    }

    if (path_vel > getAccelerationMaxPathVelocity(path_pos) || path_vel > getVelocityMaxPathVelocity(path_pos))
    {
      TrajectoryStep overshoot = trajectory.back();
      trajectory.pop_back();
      double before = trajectory.back().path_pos_;
      double before_path_vel = trajectory.back().path_vel_;
      double after = overshoot.path_pos_;
      double after_path_vel = overshoot.path_vel_;

      // CRITICAL: Binary search with EPS tolerance
      while (after - before > EPS)
      {
        const double midpoint = 0.5 * (before + after);
        double midpoint_path_vel = 0.5 * (before_path_vel + after_path_vel);

        if (midpoint_path_vel > getVelocityMaxPathVelocity(midpoint) &&
            getMinMaxPhaseSlope(before, getVelocityMaxPathVelocity(before), false) <=
                getVelocityMaxPathVelocityDeriv(before))
        {
          midpoint_path_vel = getVelocityMaxPathVelocity(midpoint);
        }

        if (midpoint_path_vel > getAccelerationMaxPathVelocity(midpoint) ||
            midpoint_path_vel > getVelocityMaxPathVelocity(midpoint))
        {
          after = midpoint;
          after_path_vel = midpoint_path_vel;
        }
        else
        {
          before = midpoint;
          before_path_vel = midpoint_path_vel;
        }
      }
      trajectory.push_back(TrajectoryStep(before, before_path_vel));

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
        if (getMinMaxPhaseSlope(trajectory.back().path_pos_, trajectory_.back().path_vel_, false) >
            getVelocityMaxPathVelocityDeriv(trajectory_.back().path_pos_))
        {
          return false;
        }
      }
    }
  }
}

// ============================================================================
// INTEGRATEBACKWARD FUNCTION (lines ~952-1020)
// ============================================================================

void Trajectory::integrateBackward(std::list<TrajectoryStep>& start_trajectory, double path_pos, double path_vel,
                                   double acceleration)
{
  std::list<TrajectoryStep>::iterator start2 = start_trajectory.end();
  --start2;
  std::list<TrajectoryStep>::iterator start1 = start2;
  --start1;
  std::list<TrajectoryStep> trajectory;
  double slope;
  assert(start1->path_pos_ <= path_pos);

  while (start1 != start_trajectory.begin() || path_pos >= 0.0)
  {
    if (start1->path_pos_ <= path_pos)
    {
      trajectory.push_front(TrajectoryStep(path_pos, path_vel));
      path_vel -= time_step_ * acceleration;
      path_pos -= time_step_ * 0.5 * (path_vel + trajectory.front().path_vel_);
      acceleration = getMinMaxPathAcceleration(path_pos, path_vel, false);
      slope = (trajectory.front().path_vel_ - path_vel) / (trajectory.front().path_pos_ - path_pos);

      if (path_vel < 0.0)
      {
        valid_ = false;
        RCLCPP_ERROR(getLogger(), "Error while integrating backward: Negative path velocity");
        end_trajectory_ = trajectory;
        return;
      }
    }
    else
    {
      --start1;
      --start2;
    }

    const double start_slope = (start2->path_vel_ - start1->path_vel_) / (start2->path_pos_ - start1->path_pos_);
    const double intersection_path_pos =
        (start1->path_vel_ - path_vel + slope * path_pos - start_slope * start1->path_pos_) / (slope - start_slope);

    // CRITICAL: Intersection check with EPS tolerance on both sides
    if (std::max(start1->path_pos_, path_pos) - EPS <= intersection_path_pos &&
        intersection_path_pos <= EPS + std::min(start2->path_pos_, trajectory.front().path_pos_))
    {
      const double intersection_path_vel =
          start1->path_vel_ + start_slope * (intersection_path_pos - start1->path_pos_);
      start_trajectory.erase(start2, start_trajectory.end());
      start_trajectory.push_back(TrajectoryStep(intersection_path_pos, intersection_path_vel));
      start_trajectory.splice(start_trajectory.end(), trajectory);
      return;
    }
  }

  valid_ = false;
  RCLCPP_ERROR(getLogger(), "Error while integrating backward: Did not hit start trajectory");
  end_trajectory_ = trajectory;
}

// ============================================================================
// SUMMARY OF ALL NUMERICAL STABILITY CHECKS
// ============================================================================

/*
1. CircularPathSegment constructor:
   - norm() < 0.000001 for segment length checks
   - dot product > 0.999999 || < -0.999999 for parallel/antiparallel detection

2. integrateForward:
   - path_pos - next_discontinuity->first < EPS for discontinuity approach
   - while (after - before > EPS) for binary search convergence

3. integrateBackward:
   - std::max(...) - EPS <= intersection_path_pos for lower bound
   - intersection_path_pos <= EPS + std::min(...) for upper bound

4. Other constants:
   - EPS = 1e-6 (main epsilon)
   - angle_tolerance = 1e-05
   - norm threshold = 0.000001
   - dot product threshold = 0.999999
*/
