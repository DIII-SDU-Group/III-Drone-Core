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

The canonical HIL/simulation profile uses a single fixed-target, jerk-bounded quintic for CableTakeoff. The legacy CableTakeoff MPC setting remains unchanged for the real profile and is not qualified against this jerk-bounded continuity contract.

## OptiTrack Profile (`opti_track`)

The SDU OptiTrack lab has no cable, so `opti_track` is a reduced flight-basics profile. A node's runtime profile is its `iii_runtime_profile` parameter if non-empty, else `III_SYSTEM_PROFILE` (set by Supervision); an empty or unknown profile is unrestricted.

### Maneuvers

Under `opti_track` the maneuver controller serves only `hover`, `fly_to_position` and `follow_waypoint_path` (an allowlist in `ManeuverAvailableInProfile()`). The other action servers stay up and reject every goal immediately with the ERROR `Maneuver <name> is not available in the opti_track profile`.

### Pose Relay (`opti_track_pose_relay`)

```bash
ros2 run iii_drone_core opti_track_pose_relay --ros-args --params-file <parameter file>
```

The relay feeds the motion-capture pose of one rigid body to PX4's EKF2 as external vision. Core keeps taking its pose from PX4 odometry, never from the motion-capture topic.

- Lab side: a second rclcpp context in the lab's ROS domain subscribes `/body_splitter/body_<rigid_body_id>/pose` (`geometry_msgs/PoseStamped` from the lab gateway Pi; best effort, volatile, keep last 1). Discovery uses the process's DDS settings, so the gateway must be reachable on the lab Wi-Fi (e.g. `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`).
- Stack side (the process's own domain): `px4_msgs/VehicleOdometry` on `/fmu/in/vehicle_visual_odometry` (best effort), health on `/opti_track/pose_relay/health`, the readiness heartbeat on `/opti_track/pose_relay/fresh`, the origin command on `/fmu/in/vehicle_command`.

Frames: the lab world is assumed Z up and bodies forward-left-up. Both are rotated by 180 degrees about x into NED/FRD: position `(x, -y, -z)`, quaternion `(w, x, -y, -z)` normalised, `pose_frame` NED. Lab +x becomes north and EKF2 takes the vision yaw as heading. **Verify this at the lab before the first flight**: with the vehicle on the floor, PX4's `/fmu/out/vehicle_odometry` must show north increasing when it is moved along lab +x, east increasing along lab -y, down decreasing when it is lifted, and yaw near 0 when its nose points along lab +x (+90 degrees towards lab -y). A Y-up stream or another body convention needs a different conversion.

Each pose is converted on arrival and forwarded at once (no buffering): on average at most `output_rate_hz`, never within half a period of the previous one, and never once it is older than `stale_timeout_s`. After a gap the stream resumes with the first fresh pose, without a catch-up burst. Non-finite poses and quaternions whose norm is not within 0.1 of 1 are rejected and counted. Velocity is unknown (`VELOCITY_FRAME_UNKNOWN`, NaN velocity, angular velocity and velocity variance), quality and reset counter are 0, and `timestamp` = `timestamp_sample` = the arrival time (system clock, us). With `UXRCE_DDS_SYNCT=0` PX4 replaces both with its own arrival time; `EKF2_EV_DELAY` covers the latency of the chain.

EKF global origin: Core's awareness handler accepts PX4 local position only with a global origin (`xy_global`, `z_global`). While `send_origin` is set and PX4 is disarmed, has no origin (`vehicle_local_position.xy_global` false) and EKF2 intends to fuse vision position (`estimator_status_flags.cs_ev_pos`), each known from a sample at most 3 s old, the relay sends `VEHICLE_CMD_SET_GPS_GLOBAL_ORIGIN` (param5 latitude, param6 longitude, param7 altitude) at most every 5 s. It never sends while armed or while PX4 reports `xy_global`, and keeps evaluating for as long as it runs, so PX4 and its agent may come up after the relay.

Parameters (read once at startup, read-only). An unset rigid-body ID, a value out of range or a mistyped override (e.g. `50` for a double) does not end the process, since a supervised restart would only repeat it: the relay logs the error once, publishes ERROR health naming every invalid parameter, and has no lab side, odometry, heartbeat or origin command.

| Parameter (`/opti_track/pose_relay/...`) | Type | Default | Valid |
| --- | --- | --- | --- |
| `rigid_body_id` | int | -1 (unset) | >= 0, the Motive rigid-body ID |
| `lab_ros_domain_id` | int | 0 | [0, 232] |
| `output_rate_hz` | double | 50.0 | [1, 200] |
| `stale_timeout_s` | double | 0.15 | (0, 1] |
| `position_variance_m2` | double | 0.0001 | (0, 1] |
| `orientation_variance_rad2` | double | 0.0004 | (0, 1] |
| `send_origin` | bool | true | |
| `origin_latitude_deg` | double | 55.3672 | [-90, 90] |
| `origin_longitude_deg` | double | 10.4310 | [-180, 180] |
| `origin_altitude_m` | double | 20.0 | [-500, 9000] |

Readiness (`std_msgs/Header` on `/opti_track/pose_relay/fresh`, `frame_id` `opti_track_pose_relay`, stamped now): 2 Hz while a pose was forwarded within `stale_timeout_s`, nothing otherwise. Supervision keys the relay's readiness on it rather than on the odometry stream.

Health (`diagnostic_msgs/DiagnosticStatus` named `opti_track_pose_relay`, always 2 Hz): ERROR for an invalid configuration, before the first pose, or when the last one is older than `stale_timeout_s`; WARN when, during the last period, the stream had a longer gap, its rate was below half `output_rate_hz`, or poses were rejected; OK otherwise. Values: `input_rate_hz`, `output_rate_hz`, `last_input_age_ms`, `max_input_gap_ms` (since the previous message), `lab_stamp_age_ms` (arrival minus header stamp; informative only, the clocks differ), `stale`, `origin_sent`, `rigid_body_id`, `rejected_samples`; `nan` where unknown.

## Tests

The current test suite covers:

- utility math/type conversions and timestamp/history semantics
- control objects such as `Reference` and reference trajectories
- basic and perception adapter serialization paths
- combined-awareness handler behavior
- the opti_track maneuver allowlist and the OptiTrack pose relay (frame conversion, output gate, health, origin decision, an end-to-end relay through both contexts)

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
