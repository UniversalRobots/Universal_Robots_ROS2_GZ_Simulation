:github_url: https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation/blob/ros2/ur_simulation_gz/doc/index.rst

ur_simulation_gz
================

This package contains configurations and example files for Gazebo simulation of Universal Robots manipulators.

Structure of the repository
---------------------------

To set up the simulation the used files are:

- ``urdf/ur_gz.ros2_control.xacro`` - macro for ros2_control configuration, defining the initial joint positions and the hardware interface plugin for the simulation
- ``urdf/ur_gz.urdf.xacro`` - main file that contains the robot description, defines reference for the Gazebo world and initializes ros2_control Gazebo plugin.

.. toctree::
   :maxdepth: 2
   :caption: Contents:

   installation
   usage
