:github_url: https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation/blob/ros2/ur_simulation_gz/doc/installation.rst

Installation
============

Skip any of below steps is not applicable.

Setup ROS2 Workspace
--------------------

1. Create a colcon workspace:

.. code-block:: console

   $ export COLCON_WS=~/workspaces/ur_gz
   $ mkdir -p $COLCON_WS/src

.. note::
   Feel free to change ``~/workspaces/ur_gz`` to whatever absolute path you want.

.. note::

   Over time you will probably have multiple ROS workspaces, so it makes sense to them all in a subfolder.

   Also, it is good practice to put the ROS version in the name of the workspace, for different tests you could just add a suffix to the base name ``ur_gz``.

2. Download the required repositories and install package dependencies:

.. code-block:: console

    $ cd $COLCON_WS
    $ git clone -b ros2 https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation.git src/ur_simulation_gz
    $ vcs import --input src/ur_simulation_gz/ur_simulation_gz-not-released.rolling.repos src # only required for rolling
    $ rosdep update && rosdep install --ignore-src --from-paths src -y

Configure and build Workspace
-----------------------------

To configure and build the workspace execute following commands:

.. code-block:: console

   $ source /opt/ros/rolling/setup.bash # necessary after installing gz-sim-vendor
   $ cd $COLCON_WS
   $ colcon build --symlink-install

and finally source your workspace before launching anything else

.. code-block:: console

    $ source $COLCON_WS/install/setup.bash
