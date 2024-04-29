Universal_Robots_ROS2_GZ_Simulation
==========================================

Example files and configurations for Gazebo simulation of Universal Robots' manipulators.

## Build status
<table width="100%">
  <tr>
    <th></th>
    <th>Humble</th>
    <th>Iron</th>
    <th>Rolling</th>
  </tr>
  <tr>
    <th>Branch</th>
    <td><a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/tree/humble">humble</a></td>
    <td><a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/tree/iron">iron</a></td>
    <td><a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/tree/ros2">ros2</a></td>
  </tr>
  <tr>
    <th>Build status</th>
    <td>
      <a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_Ignition_Simulation/actions/workflows/humble-binary-main.yml?query=event%3Aschedule++">
         <img src="https://github.com/UniversalRobots/Universal_Robots_ROS2_Ignition_Simulation/actions/workflows/humble-binary-main.yml/badge.svg?event=schedule"
              alt="Humble Binary Main"/>
      </a> <br />
    </td>
    <td>
      <a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_Ignition_Simulation/actions/workflows/iron-binary-main.yml?query=event%3Aschedule++">
         <img src="https://github.com/UniversalRobots/Universal_Robots_ROS2_Ignition_Simulation/actions/workflows/iron-binary-main.yml/badge.svg?event=schedule"
              alt="Iron Binary Main"/>
      </a> <br />
    </td>
    <td>
      <a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_Ignition_Simulation/actions/workflows/rolling-binary-main.yml?query=event%3Aschedule++">
         <img src="https://github.com/UniversalRobots/Universal_Robots_ROS2_Ignition_Simulation/actions/workflows/rolling-binary-main.yml/badge.svg?event=schedule"
              alt="Rolling Binary Main"/>
      </a> <br />
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
   git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Ignition_Simulation.git src/ur_simulation_gz
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
ros2 launch ur_robot_driver test_joint_trajectory_controller.launch.py
```

Example using MoveIt with simulated robot:
```
ros2 launch ur_simulation_gz ur_sim_moveit.launch.py
```


## Using a custom tf_prefix

In case you would like to use a custom prefix to your simulated robot, you can provide one by
passing the `tf_prefix` to the control launchfile:

``` bash
ros2 launch ur_simulation_gz ur_sim_control.launch.py tf_prefix:=my_ur_
```

However, when doing so, you will have to provide your own controller configuration file. A
corresponding file to the prefix above could look as in [this example](ur_simulation_gz/config/my_ur_controllers.yaml).

Therefore, the following would work:
```bash
ros2 launch ur_simulation_gz ur_sim_control.launch.py tf_prefix:=my_ur_ \
  controllers_file:=my_ur_controllers.yaml
```

Using the `$(var tf_prefix)` notation as in the driver does not work here, since the file is loaded
differently, where (to our knowledge) no variable substitution is possible.
