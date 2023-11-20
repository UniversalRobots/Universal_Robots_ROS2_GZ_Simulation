Universal_Robots_ROS2_GZ_Simulation
==========================================

Example files and configurations for Gazebo simulation of Universal Robots' manipulators.

## Build status

ROS2 Distro | Branch | Build status | Released packages
:---------: | :----: | :----------: | :---------------:
**Galactic** | [`galactic`](https://github.com/UniversalRobots/Universal_Robots_ROS2_Ignition_Simulation/tree/galactic) | [![Galactic Binary Build](https://github.com/UniversalRobots/Universal_Robots_ROS2_Ignition_Simulation/actions/workflows/galactic-binary-build.yml/badge.svg?branch=ros2)](https://github.com/UniversalRobots/Universal_Robots_ROS2_Ignition_Simulation/actions/workflows/galactic-binary-build.yml?branch=ros2) <br /> [![Galactic Semi-Binary Build](https://github.com/UniversalRobots/Universal_Robots_ROS2_Ignition_Simulation/actions/workflows/galactic-semi-binary-build.yml/badge.svg?branch=ros2)](https://github.com/UniversalRobots/Universal_Robots_ROS2_Ignition_Simulation/actions/workflows/galactic-semi-binary-build.yml?branch=ros2) <br /> [![Galactic Source Build](https://github.com/UniversalRobots/Universal_Robots_ROS2_Ignition_Simulation/actions/workflows/galactic-source-build.yml/badge.svg?branch=ros2)](https://github.com/UniversalRobots/Universal_Robots_ROS2_Ignition_Simulation/actions/workflows/galactic-source-build.yml?branch=ros2) | [ur_simulation_ignition](https://index.ros.org/p/ur_simulation_ignition/#galactic)
**Rolling** | [`rolling`](https://github.com/UniversalRobots/Universal_Robots_ROS2_Ignition_Simulation/tree/rolling) | [![Rolling Binary Build](https://github.com/UniversalRobots/Universal_Robots_ROS2_Ignition_Simulation/actions/workflows/rolling-binary-build.yml/badge.svg?branch=ros2)](https://github.com/UniversalRobots/Universal_Robots_ROS2_Ignition_Simulation/actions/workflows/rolling-binary-build.yml?branch=ros2) <br /> [![Rolling Semi-Binary Build](https://github.com/UniversalRobots/Universal_Robots_ROS2_Ignition_Simulation/actions/workflows/rolling-semi-binary-build.yml/badge.svg?branch=ros2)](https://github.com/UniversalRobots/Universal_Robots_ROS2_Ignition_Simulation/actions/workflows/rolling-semi-binary-build.yml?branch=ros2) <br /> [![Rolling Source Build](https://github.com/UniversalRobots/Universal_Robots_ROS2_Ignition_Simulation/actions/workflows/rolling-source-build.yml/badge.svg?branch=ros2)](https://github.com/UniversalRobots/Universal_Robots_ROS2_Ignition_Simulation/actions/workflows/rolling-source-build.yml?branch=ros2) | [ur_simulation_ignition](https://index.ros.org/p/ur_simulation_ignition/#rolling)


# Using the repository
Skip any of below steps is not applicable.

### Setup ROS Workspace

1. Create a colcon workspace:
   ```
   export COLCON_WS=~/workspaces/ur_gz
   mkdir -p $COLCON_WS/src
   ```

   > **NOTE:** Feel free to change `~/workspaces/ur_gz` to whatever absolute path you want.

   > **NOTE:** Over time you will probably have multiple ROS workspaces, so it makes sense to them all in a subfolder.
     Also, it is good practice to put the ROS version in the name of the workspace, for different tests you could just add a suffix to the base name `ur_gz`.

1. Download the required repositories and install package dependencies:
   ```
   cd $COLCON_WS
   git clone git@github.com:UniversalRobots/Universal_Robots_ROS2_Ignition_Simulation.git src/ur_simulation_gz
   vcs import src --input src/Universal_Robots_ROS2_Ignition_Simulation/ur_simulation_gz.<ros-distro>.repos
   rosdep install --ignore-src --from-paths src -y
   cd ..
   ```



### Configure and Build Workspace:
To configure and build workspace execute following commands:
  ```
  cd $COLCON_WS
  colcon build --symlink-install --mixin rel-with-deb-info compile-commands ccache
  ```

## Running Executable
```
ros2 launch ur_simulation_gz ur_sim_control.launch.py
```

Move robot using test script from  `ur_robot_driver` package (if you've installed that one):
```
ros2 launch ur_robot_driver test_joint_trajectory_controller.launch.py
```

Example using MoveIt with simulated robot:
```
ros2 launch ur_simulation_gz ur_sim_moveit.launch.py
```
