# III-Drone-Core

`iii_drone_core` is the main runtime library package for the III system. It holds the shared math/types layer, ROS message adapters, control-domain data structures, and perception/control utilities that other III packages build on.

## Package Role

This package provides:

- strongly typed math, pose, transform, and timestamp helpers
- adapters between internal domain models and `iii_drone_interfaces` ROS messages
- control-side domain objects such as `State`, `Reference`, and `ReferenceTrajectory`
- perception-side representations for point clouds, lines, powerlines, and target transforms
- reusable logic that higher-level packages use for mission execution, supervision, simulation, and ground control

## Directory Layout

### `src/utils`

- `math.cpp`: quaternion, Euler-angle, transform-matrix, projection, and vector math helpers
- `types.cpp`: conversions between Eigen-like internal types and ROS geometry/message types
- `timestamp.cpp`: timestamp wrapper utilities used by time-stamped domain models

These files define the lowest-level primitives in the package. If a module needs positions, orientations, transforms, or conversion between ROS messages and internal representations, it usually depends on this layer.

### `src/control`

- `state.cpp`: the current drone state model
- `reference.cpp`: a time-stamped reference state used by controllers and mission logic
- `reference_trajectory.cpp`: ordered reference sequences for controllers and planners
- `trajectory_generator.cpp`: trajectory generation helpers
- `trajectory_generator_client.cpp`: client-side integration with trajectory generation services
- `trajectory_interpolator.cpp`: interpolation utilities for reference trajectories
- `combined_drone_awareness_handler.cpp`: combines state, target, and awareness data into a controller-facing view

This layer is the bridge between perception/system status and the motion-control side of the stack.

### `src/adapters`

- `state_adapter.cpp`: converts `State` objects to and from ROS messages
- `reference_adapter.cpp`: converts `Reference` objects to and from ROS messages
- `reference_trajectory_adapter.cpp`: converts reference trajectory collections to and from ROS messages/path messages
- `projection_plane_adapter.cpp`: serializes projection-plane representations
- `target_adapter.cpp`: serializes target identity and target-transform information
- `point_cloud_adapter.cpp`: converts point cloud data between ROS and internal structures
- `single_line_adapter.cpp`: converts a single detected line representation
- `powerline_adapter.cpp`: converts grouped powerline detections
- `maneuver_adapter.cpp`: serializes maneuver requests/status objects
- `combined_drone_awareness_adapter.cpp`: serializes aggregated awareness data
- `gripper_status_adapter.cpp`: serializes payload/gripper state

Adapters isolate ROS interface details from the rest of the core logic. Most higher-level packages should prefer going through these adapters instead of constructing interface messages manually.

### `src/perception`

- `single_line.cpp`: single-line geometric representation and helpers
- `powerline.cpp`: grouped powerline representation and update logic
- `powerline_direction.cpp`: powerline direction estimation logic
- `hough_transformer.cpp`: Hough-transform based perception support

This code owns the geometric models consumed by maneuvering and awareness logic. It is intentionally close to the math/type layer because it relies on the same transform and quaternion utilities.

### `iii_drone_core/utils`

- `math.py`: Python equivalents of common quaternion and transform helpers used by Python tools and the GUI

This is a narrow Python helper surface, mainly intended for the GC/UI side.

## Key Concepts

### Internal Types vs ROS Messages

The package distinguishes between:

- internal domain objects: compact control/perception models used by algorithms
- ROS messages: transport-oriented representations defined in `iii_drone_interfaces`

The adapter layer exists specifically to keep those two concerns separate.

### Transform-Centric Data Flow

Most perception and target-related modules pass data around as transform matrices, quaternions, and fixed-frame references. That makes frame handling explicit and keeps conversions centralized in the utils/adapters layers.

### Shared Control Vocabulary

`State`, `Reference`, and `ReferenceTrajectory` are the common language used by controllers, mission logic, and parts of the CLI/GC stack. If you need to understand how the rest of the system reasons about motion, start there.

## Tests

The current test suite covers:

- utility math/type conversions and timestamp/history semantics
- control objects such as `Reference` and reference trajectories
- basic and perception adapter serialization paths
- combined-awareness handler behavior

Typical package-only commands:

```bash
colcon build --packages-select iii_drone_core
colcon test --packages-select iii_drone_core --ctest-args --output-on-failure
colcon test-result --verbose
```

## How To Extend This Package

- add new message transport only through an adapter, not directly inside control or perception logic
- keep frame and transform conversions centralized in `utils` and adapter helpers
- add tests when changing math or serialization behavior, because downstream packages rely on these semantics heavily
- document new control/perception models in this README when they become part of the shared package surface
