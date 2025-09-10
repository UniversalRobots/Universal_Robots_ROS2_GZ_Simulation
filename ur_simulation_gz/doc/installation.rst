:github_url: https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation/blob/ros2/ur_simulation_gz/doc/installation.rst

Installation
============

You can install the Universal Robots ROS2 GZ Simulation package in two ways:

1. **From binary packages**: This is the recommended way for most users, as it is quick and easy.
2. **From source**: This is useful if you want to modify the code or if you require to have a
   version that is newer than the latest binary package available.

Installation from binary packages
---------------------------------

1. `Install ROS2 <https://docs.ros.org/en/rolling/Installation/Ubuntu-Install-Debians.html>`_
2. Install the package using

.. code-block:: bash

  sudo apt-get install ros-${ROS_DISTRO}-ur-simulation-gz

That's it! You can now use the Universal Robots ROS2 GZ Simulation package in your ROS 2
environment. See the :ref:`ur_simulation_gz_usage` section for how to use it.


Installation from source
------------------------

.. note::
   This section is only required if you want to build the package from source. Please do that
   only, if that's required for you. Reasons why you would need to build it from source are:

   - You built one of multiple of this package's dependencies from source and want to use it with
     this package.
   - You want to develop this package or contribute to it. If you only want to modify the launch
     files to fit your application, a better approeach would be to create your own package with
     your own launch files that include the ones from this package.

   To rather install the binary packages, please refer to the `Installation from binary
   packages`_ section above.

Setup ROS2 Workspace
~~~~~~~~~~~~~~~~~~~~

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
    $ rosdep update && rosdep install --ignore-src --from-paths src -y

Configure and build Workspace
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

To configure and build the workspace execute following commands:

.. code-block:: console

   $ source /opt/ros/rolling/setup.bash # necessary after installing gz-sim-vendor
   $ cd $COLCON_WS
   $ colcon build --symlink-install

and finally source your workspace before launching anything else

.. code-block:: console

    $ source $COLCON_WS/install/setup.bash
