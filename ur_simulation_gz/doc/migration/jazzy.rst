:github_url: https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation/blob/ros2/ur_simulation_gz/doc/migration/jazzy.rst

ur_simulation_gz
^^^^^^^^^^^^^^^^

Updated argument name for tf_prefix
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

To have coherency between the driver and the GZ simulation, the launch argument to use tf_prefix has been changed from ``prefix`` to ``tf_prefix``. Moreover, differently from the driver, using such argument for this package requires the user to also provide their own controllers file, as explained in :ref:`tf_prefix for GZ <tf_prefix_gz>`.

Update arguments for MoveIt!
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

As part of the restructuring of the ``ur_moveit_config`` pkg, all the arguments that were previously passed to ``ur_moveit.launch.py`` (e.g. ``description_file``, ``safety_limits`` or ``prefix``) have been removed, in favor of the only ``moveit_launch_file`` argument.

This is due to the underlying assumption for the specified launch file to be part of a custom moveit_config pkg that already contains the correct references to a custom description pkg and related parameters.

In particular, the
option of specifying a ``tf_prefix`` for the ``ur_moveit_config`` has been removed, hence if it is needed in a GZ simulation with MoveIt!, it is expected to be handled by the user through a proper definition for it in a custom moveit_config.

Enforce absolute paths in launchfiles
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

All launchfiles now expect an absolute path for files that should be used to alter the launch
process (e.g. description file, controllers file). Before it was expecting e.g. a
``description_package`` and a ``description_file`` argument with a relative path to the package.

The default files have not been changed, so unless you specified your custom package / file
combinations, you won't need to update that to an absolute path.

Absolute paths can still be generated dynamically using a package + relative path structure inside
other launchfiles or by using ``ros2 pkg prefix`` on the command line. For example, you can do

.. code-block:: console

   $ ros2 launch ur_gz_simulation ur_sim_control.launch.py ur_type:=ur20 \
     controllers_file:=$(ros2 pkg prefix my_robot_cell_control)/share/my_robot_cell_control/config/ros2_controllers.yaml
