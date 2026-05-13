Universal_Robots_ROS2_GZ_Simulation
==========================================

Example files and configurations for Gazebo simulation of Universal Robots' manipulators.

## Build status
<table width="100%">
  <tr>
    <th></th>
    <th>Humble</th>
    <th>Jazzy</th>
    <th>Kilted</th>
    <th>Lyrical</th>
    <th>Rolling</th>
  </tr>
  <tr>
    <th>Branch</th>
    <td><a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation/tree/humble">humble</a></td>
    <td><a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation/tree/kilted">kilted</a></td>
    <td><a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation/tree/kilted">kilted</a></td>
    <td><a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation/tree/ros2">ros2</a></td>
    <td><a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation/tree/ros2">ros2</a></td>
  </tr>
  <tr>
    <th>Build status</th>
    <td>
      <a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation/actions/workflows/humble-binary-main.yml?query=event%3Aschedule++">
         <img src="https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation/actions/workflows/humble-binary-main.yml/badge.svg?event=schedule"
              alt="Humble Binary Main"/>
      </a> <br />
    </td>
    <td>
      <a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation/actions/workflows/jazzy-binary-main.yml?query=event%3Aschedule++">
         <img src="https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation/actions/workflows/jazzy-binary-main.yml/badge.svg?event=schedule"
              alt="Jazzy Binary Main"/>
      </a> <br />
    </td>
    <td> <!-- Kilted -->
      <a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation/actions/workflows/kilted-binary-main.yml?query=event%3Aschedule++">
         <img src="https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation/actions/workflows/kilted-binary-main.yml/badge.svg?event=schedule"
              alt="Kilted Binary Main"/>
      </a> <br />
    </td>
    <td> <!-- Lyrical -->
      <a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation/actions/workflows/lyrical-binary-main.yml?query=event%3Aschedule++">
         <img src="https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation/actions/workflows/lyrical-binary-main.yml/badge.svg?event=schedule"
              alt="Lyrical Binary Main"/>
      </a> <br />
    </td>
    <td>
      <a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation/actions/workflows/rolling-binary-main.yml?query=event%3Aschedule++">
         <img src="https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation/actions/workflows/rolling-binary-main.yml/badge.svg?event=schedule"
              alt="Rolling Binary Main"/>
      </a> <br />
    </td>
  </tr>
  <tr>
    <th>Buildfarm</th>
    <td>
      <a href='https://build.ros2.org/job/Hbin_uJ64__ur_simulation_gz__ubuntu_jammy_amd64__binary/'><img src='https://build.ros2.org/job/Hbin_uJ64__ur_simulation_gz__ubuntu_jammy_amd64__binary/badge/icon?subject=uJ64_ur_simulation_gz'></a>
    </td>
    <td>
      <a href='https://build.ros2.org/job/Jbin_uN64__ur_simulation_gz__ubuntu_noble_amd64__binary/'><img src='https://build.ros2.org/job/Jbin_uN64__ur_simulation_gz__ubuntu_noble_amd64__binary/badge/icon?subject=uN64_ur_simulation_gz'></a>
    </td>
    <td>
      <a href='https://build.ros2.org/job/Kbin_uN64__ur_simulation_gz__ubuntu_noble_amd64__binary/'><img src='https://build.ros2.org/job/Kbin_uN64__ur_simulation_gz__ubuntu_noble_amd64__binary/badge/icon?subject=uN64_ur_simulation_gz'></a>
    </td>
    <td>
      <a href='https://build.ros2.org/job/Lbin_uR64__ur_simulation_gz__ubuntu_resolute_amd64__binary/'><img src='https://build.ros2.org/job/Lbin_uR64__ur_simulation_gz__ubuntu_resolute_amd64__binary/badge/icon?subject=uR64_ur_simulation_gz'></a>
    </td>
    <td>
      <a href='https://build.ros2.org/job/Rbin_uR64__ur_simulation_gz__ubuntu_resolute_amd64__binary/'><img src='https://build.ros2.org/job/Rbin_uR64__ur_simulation_gz__ubuntu_resolute_amd64__binary/badge/icon?subject=uR64_ur_simulation_gz'></a>
    </td>
  </tr>
</table>

A more [detailed build status](ci_status.md) shows the state of all CI workflows inside this repo.
Please note that the detailed view is intended for developers, while the one here should give end
users an overview of the current released state.


## Using the repository
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
   git clone -b ros2 https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation.git src/ur_simulation_gz
   rosdep update && rosdep install --ignore-src --from-paths src -y
   ```



### Configure and Build Workspace:
To configure and build workspace execute following commands:
  ```
  cd $COLCON_WS
  colcon build --symlink-install
  ```

## Running Executable
First, source your workspace

```
source $COLCON_WS/install/setup.bash
```

```
ros2 launch ur_simulation_gz ur_sim_control.launch.py
```

Move robot using test script from  `ur_robot_driver` package (if you've installed that one):
```
ros2 run ur_robot_driver example_move.py
```

Example using MoveIt with simulated robot:
```
ros2 launch ur_simulation_gz ur_sim_moveit.launch.py
```
