# TrajOpt SCO vs TrajOpt IFOPT: Implementation Comparison

This document compares the two trajectory optimization implementations in tesseract_planning:
- **TrajOpt SCO** (`tesseract_motion_planners/trajopt/`) - Original Sequential Convex Optimization
- **TrajOpt IFOPT** (`tesseract_motion_planners/trajopt_ifopt/`) - IFOPT-based reimplementation

## Differences Found

### 1. Smoothing Cost Defaults (Fixed)

| Setting | TrajOpt SCO | TrajOpt IFOPT (before fix) | TrajOpt IFOPT (after fix) |
|---|---|---|---|
| `smooth_velocities` | `true` | `true` | `true` |
| `smooth_accelerations` | `true` | `false` | `true` |
| `smooth_jerks` | `true` | `false` | `true` |

**Files:** `trajopt/include/.../trajopt_default_composite_profile.h:68-72`, `trajopt_ifopt/include/.../trajopt_ifopt_default_composite_profile.h:57-62`

SCO header comments said "Default: false" for accel/jerk but the actual C++ initializer was `true`. IFOPT matched the incorrect comments rather than the actual behavior.

### 2. Smoothing Coefficients (Fixed)

| Setting | TrajOpt SCO | TrajOpt IFOPT (before fix) | TrajOpt IFOPT (after fix) |
|---|---|---|---|
| Default velocity coeff | 5.0 per joint | empty (no fallback) | 5.0 per joint |
| Default acceleration coeff | 1.0 per joint | empty | 1.0 per joint |
| Default jerk coeff | 1.0 per joint | empty | 1.0 per joint |

**Files:** SCO: `trajopt_utils.h:92,104,116` (function parameter defaults). IFOPT: `trajopt_ifopt_default_composite_profile.cpp:112-131`.

### 3. SQP Solver Parameter Defaults (No Impact)

Both structs have identical default values for all shared parameters. Only naming differs:
- `max_iter` (SCO) vs `max_iterations` (IFOPT)
- `trust_box_size` (SCO) vs `initial_trust_box_size` (IFOPT)
- SCO has `num_threads`; IFOPT does not (single-threaded only)

### 4. Cost Penalty Type Architecture (Structural)

| Aspect | TrajOpt SCO | TrajOpt IFOPT |
|---|---|---|
| Cost list structure | Flat: `costs[]`, `constraints[]` | Categorized: `squared_costs[]`, `hinge_costs[]`, `absolute_costs[]`, `constraints[]` |
| Penalty type determination | Implicit per `TermInfo` subclass | Explicit via `CostPenaltyType` enum |
| Collision costs | `TT_COST` (internal penalty) | `kHinge` (explicit hinge penalty) |
| Cartesian costs | `TT_COST` (internal penalty) | `kSquared` (explicit squared penalty) |
| Smoothing costs | `TT_COST` (internal penalty) | `kSquared` (explicit squared penalty) |

**Files:** SCO: `trajopt_profile.h:45-49`. IFOPT: `trajopt_ifopt_profile.h:45-51`, `trajopt_ifopt_motion_planner.cpp:126-166`.

The hinge penalty on collision costs has constant gradient when in collision, which can compete with Cartesian constraint penalties at endpoints that are intentionally in collision.

### 5. Collision Constraint Granularity (Structural)

| Aspect | TrajOpt SCO | TrajOpt IFOPT |
|---|---|---|
| Structure | One monolithic `CollisionTermInfo` spanning all timesteps | N individual per-timestep/per-segment constraints |
| Gradient sparsity | Dense (single term touches all timestep variables) | Sparse (each constraint touches 1-2 timestep variables) |
| Fixed timestep handling | `collision->fixed_steps` embedded in term | Skip constraint creation for fixed indices |

**Files:** SCO: `trajopt_utils.cpp:271-285`. IFOPT: `trajopt_ifopt_utils.cpp:119-189`.

Per-timestep collision costs in IFOPT produce more focused gradient at each timestep (including endpoints), while SCO distributes collision gradients across all timesteps in one term.

### 6. QP Solver Interface (Structural)

| Aspect | TrajOpt SCO | TrajOpt IFOPT |
|---|---|---|
| QP solver wrapper | `sco::OSQPModelConfig` | `trajopt_sqp::OSQPEigenSolver` (wraps OsqpEigen) |
| Default initialization | `sco::OSQPModelConfig::setDefaultOSQPSettings()` | `trajopt_sqp::OSQPEigenSolver::setDefaultOSQPSettings()` |

Both configure the same OSQP library but through different wrappers.

### 7. SQP Solver Algorithm (Structural)

| Aspect | TrajOpt SCO | TrajOpt IFOPT |
|---|---|---|
| Optimizer class | `sco::BasicTrustRegionSQP` | `trajopt_sqp::TrustRegionSQPSolver` |
| Problem class | `trajopt::TrajOptProb` | `trajopt_sqp::TrajOptQPProblem` |
| Variable representation | Flat `DblVec` | Hierarchical `NodesVariables` |
| Multi-threading | `BasicTrustRegionSQPMultiThreaded` available | Single-threaded only |

These are independent codebases that may differ in merit function computation, constraint penalty inflation strategy, and convergence criteria.

### 8. Dynamic Cartesian Waypoint Handling (Structural)

| Aspect | TrajOpt SCO | TrajOpt IFOPT |
|---|---|---|
| Static frame handling | `CartPoseTermInfo` | `CartPosConstraint` |
| Dynamic frame handling | `DynamicCartPoseTermInfo` (separate class) | Same `CartPosConstraint` (no distinction) |

**Files:** SCO: `trajopt_default_move_profile.cpp:105-108,126-187`. IFOPT: `trajopt_ifopt_default_move_profile.cpp:102-218`.

IFOPT does not distinguish between static and dynamic Cartesian frames.

### 9. Fixed Waypoints Not Truly Fixed (Bug - Fixed)

| Aspect | TrajOpt SCO | TrajOpt IFOPT (before fix) | TrajOpt IFOPT (after fix) |
|---|---|---|---|
| `info.fixed = true` effect | Locks variables (non-optimizable) | Only skips collision costs | Locks variables via tight bounds |
| Mechanism | `pci->basic_info.fixed_timesteps` | `fixed_steps` only for collision skip | Variable bounds `[pos, pos]` |
| Variable bounds | N/A (locked) | Full joint limits | `[position, position]` |

**Files:** SCO: `trajopt_motion_planner.cpp:249-255`. IFOPT: `trajopt_ifopt_default_move_profile.cpp:244-247`.

Before the fix, IFOPT's `info.fixed = true` only affected collision cost creation. The variable remained fully optimizable with full joint-limit bounds, and the joint position "constraint" was a soft penalty that could be violated. Now, fixed waypoints have tight bounds that lock them at their position.

### 10. Missing Singularity Avoidance (Feature Gap)

SCO has `avoid_singularity` (default: `false`, coeff: 5.0). IFOPT does not implement this.

## Cartesian Endpoint Issue Analysis

When a Cartesian endpoint is in collision and collision carries only a cost (not a constraint), IFOPT may fail to reach the endpoint while SCO succeeds. This is caused by the combination of:

1. Per-timestep collision costs producing focused hinge-penalty gradient at the endpoint
2. The `trajopt_sqp::TrustRegionSQPSolver` potentially not inflating constraint penalty coefficients aggressively enough to overcome the collision cost
3. Different merit function evaluation in the external solver libraries

This is a solver-level issue in the external `trajopt_sqp` library.
