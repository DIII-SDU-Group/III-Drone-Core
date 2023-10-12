# III-Drone-ROS2-pkg
ROS2 package for the III-Drone system

## Compatibility
This version is compatible with
- `ROS2 Humble`
- [`PX4-Autopilot` DIII fork tag `v1.14.0-rc2`](https://github.com/DIII-SDU-Group/PX4-Autopilot/tree/v1.14.0-rc2)
- [`px4_msgs` DIII fork tag `v1.14`](https://github.com/DIII-SDU-Group/px4_msgs/tree/v1.14)
- [`micro-ROS-agent` DIII fork tag `III-Drone-v2.2`](https://github.com/DIII-SDU-Group/micro-ROS-Agent/tree/III-Drone-v2.2)
- [`micro_ros_msgs` DIII fork tag `III-Drone-v2.2`](https://github.com/DIII-SDU-Group/micro_ros_msgs/tree/III-Drone-v2.2)
- [`III-Drone-Interfaces` v2.2](https://github.com/DIII-SDU-Group/III-Drone-Interfaces/tree/v2.2-staging)

## Build and installation
Follow the [ROS2 Humble installation instructions](https://docs.ros.org/en/humble/Installation.html). Make sure to source the tools:
```
source /opt/ros/humble/setup.bash
```
Optionally, add it to `.bashrc`:
```
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

Pulling the core packages of the III-Drone system, `III-Drone-Core` and [`III-Drone-Interfaces`](https://github.com/DIII-SDU-Group/III-Drone-Interfaces/), as well as the [`px4_msgs`](https://github.com/DIII-SDU-Group/px4_msgs), [`micro-ROS-agent`](https://github.com/DIII-SDU-Group/micro-ROS-Agent), and [`micro_ros_msgs`](https://github.com/DIII-SDU-Group/micro_ros_msgs) packages:
```
cd <ROS2-DIII-workspace>/src

git clone git@github.com:DIII-SDU-Group/III-Drone-Core.git -b v2.2-staging
git clone git@github.com:DIII-SDU-Group/III-Drone-Interfaces.git -b v2.2-staging

git clone git@github.com:DIII-SDU-Group/px4_msgs.git -b v1.14
git clone git@github.com:DIII-SDU-Group/micro-ROS-Agent.git -b III-Drone-v2.2
git clone git@github.com:DIII-SDU-Group/micro_ros_msgs.git -b III-Drone-v2.2
```

TODO: [Make rosdep work](https://github.com/DIII-SDU-Group/III-Drone-Core/issues/24) to more easily fetch required packages.

If simulation is required, pull the [`III-Drone-Simulation`](https://github.com/DIII-SDU-Group/III-Drone-Simulation) package:
```
git clone git@github.com:DIII-SDU-Group/III-Drone-Simulation.git -b v2.2-staging
```

If ground control system is required, pull the [`III-Drone-GC`](https://github.com/DIII-SDU-Group/III-Drone-GC) package:
```
git clone git@github.com:DIII-SDU-Group/III-Drone-GC.git -b v2.2-staging
```

Build the workspace:
```
cd <ROS2-DIII-workspace>

colcon build
```

## Building PX4-Autopilot firmware
TODO: Write this section, including how to add new DDS topics.

## Contribution
Current efforts is to refactor, debug, and prepare the current version v2.1-ICRA2024 to the next version v2.2. Major structural changes are made. Tasks to be done can be found [here](https://github.com/orgs/DIII-SDU-Group/projects/1/views/1).

### Development rules
- At the moment, only pull requests to branch [`v2.2-staging`](https://github.com/DIII-SDU-Group/III-Drone-ROS2-pkg/tree/v2.2-staging) are accepted
- Pull requests need to solve an issue
- Prioritize solving issues that are listed in the [v2.2 project](https://github.com/orgs/DIII-SDU-Group/projects/1/views/1) 
- All changes should be merged into `v2.2-staging` by pull request - no direct commits except for changes to [`README.md`](https://github.com/DIII-SDU-Group/III-Drone-ROS2-pkg/blob/v2.2-staging/README.md)
- When working on an issue, open a new branch out from this branch - when finished, open a pull request for merging back into this branch
