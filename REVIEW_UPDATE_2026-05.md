# Review Update — 2026-05-11

This document consolidates corrections and additions to the earlier TOTG-vs-MoveIt2
analysis (originally written 2026-01-17). It supersedes any conflicting claim in
the older documents. Source-line references here use the **current** layout:

- Source: `time_parameterization/totg/src/time_optimal_trajectory_generation.cpp`
- Header: `time_parameterization/totg/include/tesseract/time_parameterization/totg/time_optimal_trajectory_generation.h`
- Namespace: `tesseract::time_parameterization` (the old `tesseract_common::` paths in the prior docs are stale; the project moved to nested namespaces and a single CMake project).

---

## 1. State of MoveIt2 since the original analysis

Verified May 2026. **No commits to `time_optimal_trajectory_generation.{cpp,hpp}` on `moveit/moveit2:main` since the original analysis was written.** The most recent commits touching those files are:

- `932c192` (Jan 9, 2025) — Add const specifier to moveit_core ([#3202](https://github.com/moveit/moveit2/issues/3202))
- `9766451` (Nov 29, 2024) — Header renamed `.h` → `.hpp`

One **new TOTG-relevant issue** has been filed since the analysis was written:

- **[moveit2#3565](https://github.com/moveit/moveit2/issues/3565)** (opened Sept 2, 2025) — *TOTG paths can contain extremely high accelerations.* Acceleration values of ~10¹⁵ m/s² have been observed. The root cause is division by a near-machine-epsilon `time_step` in the per-waypoint acceleration computation. **Tesseract is exposed to this**; see finding §3 below.
- [moveit2#3694](https://github.com/moveit/moveit2/issues/3694) (opened Feb 2026) — vibration during deceleration. Reporter has not isolated it to TOTG; likely jerk/config related. Tracking only.

---

## 2. Correction: "MoveIt2 has a bug at line 759, Tesseract is correct" — WRONG

This claim appears in:
- `COMPARISON_SUMMARY.md` § "MoveIt2 Bug Found"
- `TOTG_COMPARISON.md` § 3 "Critical Bug Found in MoveIt2"
- `TOTG_SIDE_BY_SIDE_COMPARISON.md` § 4 "integrateForward - CRITICAL BUG in MoveIt2"

**Actual current Tesseract code** (`time_optimal_trajectory_generation.cpp:758-764`):

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

Tesseract uses `trajectory_.back()` (class member) in the else branch — exactly the same mixed usage the prior analysis flagged as a "MoveIt2 bug." MoveIt2 main today also has this exact code.

**Why it isn't actually a bug in either project:** `integrateForward` is invoked as `integrateForward(trajectory_, after_acceleration)` (line 460). The parameter `trajectory` and the member `trajectory_` reference the same `std::list<TrajectoryStep>` during execution. The stylistic inconsistency is harmless. There is no stale-data hazard.

**Implications for the prior analysis:**
- The "1 confirmed bug" entry in `COMPARISON_SUMMARY.md` quick-stats should be removed.
- The "MoveIt2 = 8/10" rating loses its only confirmed-bug deduction.
- The recommendation to NOT port MoveIt2 code stands on the *numerical-stability* improvements, not on this non-bug.

---

## 3. New finding — Tesseract is exposed to [moveit2#3565](https://github.com/moveit/moveit2/issues/3565) (huge accelerations)

Not covered in the prior analysis.

The vulnerability is in `Trajectory::getAcceleration` (`time_optimal_trajectory_generation.cpp:1079-1086`):

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

The `if (time_step > 0.0)` guard prevents division by exactly zero, **not** by `1e-15`. Two paths can produce a near-epsilon `time_step`:

1. **Trajectory-step timing** (constructor, line 487): `dt = (next_pos − prev_pos) / avg_vel`. After the bisection at switching points (line 743) two adjacent steps in `trajectory_` can be spatially very close, making `dt` tiny.
2. **Sub-segment query** (`getPathData`, line 1034): when queried just past a segment boundary, `data.time − data.prev_time` is tiny.

The `1e-8` floor in `assignData` lines 960-961 / 972-973 only enforces monotonicity at the **outer waypoint** level — it does not protect the internal `getPathData → getAcceleration` path.

**Symptom:** `getAcceleration` returns values orders of magnitude over the configured limits at affected waypoints.

**Severity:** 🟡 medium. Rare in practice (the same kind of trajectory shapes that trigger [#3565](https://github.com/moveit/moveit2/issues/3565) on MoveIt2), but should be on the known-limitations list.

**Mitigation options** (not implementing yet, for later discussion):
- Add a sensible floor on `time_step` in `getAcceleration` (e.g. `max(time_step, 1e-6)`) — quick but lossy.
- Compute acceleration from the segment's stored `acceleration` rather than from finite differences on tangents — more involved but mathematically clean.

---

## 4. New finding — `getTime` divides by zero in coast regions

`time_optimal_trajectory_generation.cpp:1057-1069`:

```cpp
double time_step = it->time_ - previous->time_;
const double acceleration =
    2.0 * (it->path_pos_ - previous->path_pos_ - time_step * previous->path_vel_) / (time_step * time_step);

const double a = 0.5 * acceleration;
const double b = previous->path_vel_;
const double c = previous->path_pos_ - pos;

const double d = std::pow(b, 2.0) - (4 * a * c);
const double e = ((d > 0) ? std::sqrt(d) : 0);
const double dt = (-b + e) / (2.0 * a);
assert(!(dt < 0));
return (previous->time_ + dt);
```

If two adjacent steps in `trajectory_` have equal `path_vel_` (a constant-velocity coast region — exactly the "bang-coast-bang" middle phase TOTG explicitly produces), then `acceleration = 0` → `a = 0` → line 1067 divides by zero → `dt = ±∞`. The `assert(!(dt < 0))` does not catch `+∞`.

A linear-fallback `dt = (pos − previous->path_pos_) / previous->path_vel_` is appropriate when `|a| < eps`.

**Severity:** 🟡 medium. Coast regions are common; whether this actually fires depends on whether `getTime` is called within a coast region. `assignData` calls `getTime(dist_mapping[seg_idx])` at line 959, which can land on coast segments.

---

## 5. New finding — Inconsistent zero-detection in `getAccelerationMaxPathVelocity`

`time_optimal_trajectory_generation.cpp:870-900`:

```cpp
for (unsigned int i = 0; i < joint_num_; ++i)
{
  if (config_deriv[i] != 0.0)
  {
    for (unsigned int j = i + 1; j < joint_num_; ++j)
    {
      if (config_deriv[j] != 0.0)
      {
        double a_ij = (config_deriv2[i] / config_deriv[i]) - (config_deriv2[j] / config_deriv[j]);
        if (a_ij != 0.0)
        {
          ...
        }
      }
    }
  }
  else if (config_deriv2[i] != 0.0)
  {
    max_path_velocity = std::min(max_path_velocity, sqrt(max_acceleration_[i] / std::abs(config_deriv2[i])));
  }
}
```

`getMinMaxPathAcceleration` (line 854) was praised in the original analysis for using `tesseract::common::almostEqualRelativeAndAbs(config_deriv[i], 0.0, eps)`. **The function two definitions below uses bare `!= 0.0` for the same kind of check** (lines 877, 881, 884). A `config_deriv[i]` of, say, `1e-300` passes `!= 0.0` and then divides into ~`1e+300` at line 887.

**Severity:** 🟢 low (rarely fires in practice). Easy fix — make these consistent with `getMinMaxPathAcceleration`'s approach.

---

## 6. Strengthening — antiparallel handling is worse than the prior analysis suggested

The prior docs framed the antiparallel issue as "the `acos` could return NaN, but Tesseract clamps it with `std::max(-1.0, …)`, so it's protected; only stylistically MoveIt2 is more explicit."

That's **incomplete**. Walk-through `CircularPathSegment` (line 220-273) when `start_direction ≈ -end_direction`:

- Line 238 early-exit: `(start_dir − end_dir).norm() ≈ 2.0` → does **not** trigger.
- Line 254: `angle = acos(max(-1.0, dot)) ≈ π` — acos protected.
- Line 263: `tan(0.5 * π) = ∞` → `radius = distance / ∞ = 0`.
- Line 270: `center = intersection + (end_dir − start_dir).normalized() * radius / cos(0.5 * π)` = `... * 0 / 0` = **NaN**.
- Line 271: `x` is **NaN**.
- `getConfig(s) = center + radius * (x*cos(angle) + y*sin(angle))` propagates **NaN** for any `s`.

Then in `Path::Path` (line 345-351):

- `end_config = blend_segment->getConfig(0.0)` is **NaN**.
- Line 346: `(end_config - start_config).norm() > 0.000001` evaluates to `NaN > 0.000001` → `false`. The bridging `LinearPathSegment` is **silently not added**.
- Line 351: `start_config = blend_segment->getConfig(blend_segment->getLength()) = NaN`. Subsequent linear segments are constructed from a NaN endpoint.

In other words, the `acos` clamp prevents one NaN source but the **geometry calculation itself is broken for antiparallel inputs**. The dummy-joint workaround (lines 157-173) is what actually keeps this from crashing production trajectories — by ensuring no two adjacent path directions in the augmented (n+1)-dim space are antiparallel, even when the real joints are.

**Implication:** the proposed fix (adding `start_dot_end < -0.999999` to the early-exit) is more impactful than the prior analysis suggested. It is a precondition for safely removing the dummy-joint workaround.

---

## 7. Copy-paste in error messages

`time_optimal_trajectory_generation.cpp:99,103`:

```cpp
throw std::runtime_error("IterativeSplineParameterization, velocity scale factor must be greater than zero!");
```

Both occurrences say `"IterativeSplineParameterization"` from inside TOTG code. Line 103 is about *acceleration*, not velocity. Cosmetic, but misleading when debugging.

---

## 8. Updated map: what's verified accurate in the prior analysis

Confirmed against current source:

| Claim | Verified at |
|---|---|
| `EPS = 0.000001` | line 60 |
| Single-waypoint short-circuit | lines 144-154 |
| Dummy-joint workaround for Issue [#27](https://github.com/tesseract-robotics/tesseract_planning/issues/27) | lines 157-173 |
| Scaling factors applied to limits (Issue [#118](https://github.com/tesseract-robotics/tesseract_planning/issues/118)) | lines 168-173 |
| Limit-mismatch is logged, not return-false | lines 106-109 |
| `acos(std::max(-1.0, …))` (MoveIt PR [#1861](https://github.com/moveit/moveit/issues/1861)) | line 254 |
| Switching-point dedup via EPS (MoveIt [#1665](https://github.com/moveit/moveit/issues/1665)) | lines 676-680 |
| `getMinMaxPathAcceleration` zero-tangent guard | line 854 |
| Equal-slope guard in `integrateBackward` | lines 815-821 |
| `almostEqualRelativeAndAbs` in `integrateBackward` | lines 781, 786, 825-828 |
| `TrajectoryStep` NaN asserts (header) | header lines 179-180 |
| Endpoint zero velocity/acceleration | lines 458, 474 |
| Torque-limits NOT implemented | (whole-file search) |
| Issue [#164](https://github.com/tesseract-robotics/tesseract_planning/issues/164) per-state scaling is by design | architectural |

## 9. Updated gap list

Reorganized priorities given the new findings:

| Item | Severity | Effort | Notes |
|---|---|---|---|
| Antiparallel early-exit (incl. dot < -0.999999) | 🔴 high | 🟢 low | Precondition for removing the dummy-joint workaround; resolves Issue [#27](https://github.com/tesseract-robotics/tesseract_planning/issues/27) |
| `getAcceleration` exposure to [moveit2#3565](https://github.com/moveit/moveit2/issues/3565) | 🟡 medium | 🟡 medium | Document at minimum; fix is non-trivial |
| `getTime` div-by-zero in coast regions | 🟡 medium | 🟢 low | Linear fallback when `a≈0` |
| Strict limit validation (return false on mismatch) | 🟡 medium | 🟢 low | Already covered |
| Inconsistent `!= 0.0` in `getAccelerationMaxPathVelocity` | 🟢 low | 🟢 low | Consistency fix |
| Wrong class name in error messages | 🟢 low | 🟢 trivial | Cosmetic |
| Torque limits | 🔴 high | 🔴 high | Major feature, evaluate need |

---

## 10. Things the prior analysis got wrong or under-stated, summary

1. Claimed MoveIt2 has a bug at line 759 that Tesseract fixed — both projects have the same code, and it isn't a bug (§2).
2. Framed antiparallel handling as "protected by `acos` clamp" — the geometry still produces NaN that propagates silently (§6).
3. Missed [moveit2#3565](https://github.com/moveit/moveit2/issues/3565) exposure (§3).
4. Missed `getTime` div-by-zero in coast regions (§4).
5. Missed the inconsistent zero-detection across the two `getMin*` functions (§5).
6. Mis-attributed PR numbers in the "Bug Fixes Tesseract Already Has" table — see §11.
7. Stale file paths and namespaces throughout (project was restructured in the recent rebased commits).

---

## 11. Correction — PR mis-attribution in `COMPARISON_SUMMARY.md` bug-fixes table

Verified each cited MoveIt PR against its actual title and content. Two rows in the original table were misattributed:

| Row in original table | Cited PR | What the cited PR actually does | Correction |
|---|---|---|---|
| "Division by zero (CircularPathSegment) → Partially (only parallel)" | [moveit/moveit2#1218](https://github.com/moveit/moveit2/pull/1218) | "Make TOTG the default time-parameterization algorithm" — **not** a CircularPathSegment bug fix | This row's bug description is real, but the PR# was wrong. The correct PR for this fix is [moveit/moveit#2957](https://github.com/moveit/moveit/pull/2957). |
| "Undefined behavior (deep copy) → N/A (different structure)" | [moveit/moveit#2957](https://github.com/moveit/moveit/pull/2957) | **The antiparallel/parallel division-by-zero fix in `CircularPathSegment`** — replaced the norm-difference check with `start_dot_end > 0.999999 \|\| start_dot_end < -0.999999`. Not "deep copy". | Row description was wrong AND status was wrong. The fix is what Tesseract is missing. |

**Net effect:** the prior table essentially listed the same MoveIt fix twice under two wrong descriptions, marked the one with the right PR# as "Partially fixed" and the one with the wrong description as "N/A". After correction:

- One row only: **antiparallel/parallel division-by-zero in `CircularPathSegment`**, fixed by [moveit/moveit#2957](https://github.com/moveit/moveit/pull/2957), **MISSING in Tesseract**.

The fix from moveit#2957 (replace the norm-difference parallel check with the dot-product bounds check) is exactly the 5-line change discussed throughout the analysis docs as the antiparallel fix. The prior analysis had identified the gap, but didn't realize MoveIt had also explicitly fixed it via this PR.

### Other label corrections from verification

- [moveit/moveit2#2741](https://github.com/moveit/moveit2/issues/2741) is "Pilz Industrial Motion Planner blend generates duplicate `time_from_start`" — a Pilz planner issue, **not** a TOTG issue. The original docs labeled it "Blend radius duplicate timestamps" which is misleading.
- [moveit/moveit#809](https://github.com/moveit/moveit/pull/809) is "Add time-optimal trajectory parameterization plugin" — **closed, not merged**. The TOTG plugin was actually merged via #1365. The reference still resolves, just worth knowing the linked PR isn't the merge.
- [moveit/moveit2#3504](https://github.com/moveit/moveit2/issues/3504) is closed as "not planned" — it's about `RobotModelLoader::jointBoundsFromURDF` not loading acceleration limits, not strictly a TOTG bug.
