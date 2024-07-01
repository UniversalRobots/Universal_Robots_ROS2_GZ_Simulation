ur_simulation_gz
================

This package contains configurations and example files for Gazebo simulation of Universal Robots manipulators.

Structure of the repository
---------------------------

To set up the simulation the used files are:

- ``urdf/ur_gz.ros2_control.xacro`` - macro for ros2_control configuration, defining the initial joint positions and the hardware interface plugin for the simulation
- ``urdf/ur_gz.urdf.xacro`` - main file that contains the robot description, defines reference for the Gazebo world and initializes ros2_control Gazebo plugin.

Installation
------------

Skip any of below steps is not applicable.

Setup ROS2 Workspace
^^^^^^^^^^^^^^^^^^^^

1. Create a colcon workspace:

.. code-block:: console

   $ export COLCON_WS=~/workspaces/ur_gz
   $ mkdir -p $COLCON_WS/src

.. note::
   Feel free to change `~/workspaces/ur_gz` to whatever absolute path you want.

.. note::
   Over time you will probably have multiple ROS workspaces, so it makes sense to them all in a subfolder.
     Also, it is good practice to put the ROS version in the name of the workspace, for different tests you could just add a suffix to the base name `ur_gz`.

2. Download the required repositories and install package dependencies:

.. code-block:: console

    $ cd $COLCON_WS
    $ git clone -b ros2 https://github.com/UniversalRobots/Universal_Robots_ROS2_Ignition_Simulation.git src/ur_simulation_gz
    $ rosdep update && rosdep install --ignore-src --from-paths src -y

Configure and Build Workspace
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To configure and build workspace execute following commands:

.. code-block:: console

   $ cd $COLCON_WS
   $ colcon build --symlink-install

and finally source your workspace before launching anything

.. code-block:: console

    $ source $COLCON_WS/install/setup.bash

Usage
-----

To launch the simulation, two files can be used:

- ``launch/ur_sim_control.launch.py``
- ``launch/ur_sim_moveit.launch.py``

They both start Gazebo, but only the second launches MoveIt! together with it, allowing to plan motions using either MoveGroup interfaces or the Motion Planning panel in Rviz.

So, if only Gazebo and Rviz are needed:

.. code-block:: console

   $ ros2 launch ur_simulation_gz ur_sim_control.launch.py ur_type:=ur10e

If we also want to be able to use MoveIt!, then:

.. code-block:: console

    $ ros2 launch ur_simulation_gz ur_sim_moveit.launch.py ur_type:=ur10e

.. image:: resources/gz_simulation_moveit.png
   :width: 95%
   :alt: Gazebo with MoveIt!

.. note::

   All the additional launch arguments are described in the launch files themselves.

Customization
-------------

Beyond the default usage, the package offers some customization options.

Custom Description
^^^^^^^^^^^^^^^^^^

The first option is easily achievable by using the launch argument ``description_file``,  which allows to pass the absolute path of a custom description to both launchers. Together with it, it could be useful to choose a custom Rviz configuration file and the launch argument ``rviz_config_file`` can be passed for that. An example of their usage:

.. code-block:: console

   $ ros2 launch ur_simulation_gz ur_sim_control.launch.py ur_type:=ur10e description_file:="/home/ubuntu/ur_gz_test.urdf.xacro" rviz_config_file:="/home/ubuntu/rviz_test.rviz"

tf_prefix
^^^^^^^^^

Also here, like in the driver package, it is possible to specify tf_prefix using the ``tf_prefix`` launch argument, but for this package this is not the only step required. Since controllers loading is handled differently, it is necessary to define a custom controllers file with the desired tf_prefix. Assuming ``tf_prefix:="alice_"``, an example of such file could be:

.. literalinclude:: resources/ur_controllers_test.yaml
    :emphasize-lines: 52-57, 80-85, 107-112, 118-123

To load the newly defined file, it is possible to specify its absolute path with the ``controllers_file`` argument. Together with it the desired prefix should be also be specified as argument, like in the following example:

.. code-block:: console

   $ ros2 launch ur_simulation_gz ur_sim_control.launch.py ur_type:=ur10e tf_prefix:="alice_" controllers_file:="/home/ubuntu/ur_controllers_test.yaml"

.. note::

   The ``tf_prefix`` argument is not available for ``ur_sim_moveit.launch.py``, since it would require a custom definition of the moveit config package for properly setting it up.

Custom World
^^^^^^^^^^^^

The last customization option allows to instantiate the robot in a proper setup instead of an empty world, like the given launch files do by default. The first step to create a complete simulation is to define a world file (.sdf): for this example we can use a simple custom world ``test_world.sdf``, located it in ``ur_gz_simulation/doc/resources``. For more details about building worlds in Gazebo, it's possible to check the `related tutorial <https://gazebosim.org/docs/harmonic/sdf_worlds>`_.
To use the new world changes it's enough to specify its absolute path in the ``world_file`` argument:


.. code-block:: console

   $ ros2 launch ur_simulation_gz ur_sim_control.launch.py ur_type:=ur10e world_file:=<path_to_gz_simulation>/doc/resources/test_world.sdf

or using MoveIt!

.. code-block:: console

   $ ros2 launch ur_simulation_gz ur_sim_moveit.launch.py ur_type:=ur10e world_file:=<path_to_gz_simulation>/doc/resources/test_world.sdf

In this way, when launching the simulation, Gazebo will use the indicated custom world instead of the default empty, like in the following picture.

.. image:: resources/gz_simulation_custom_world.png
   :width: 95%
   :alt: Gazebo custom world
