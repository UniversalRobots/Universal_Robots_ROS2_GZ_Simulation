:github_url: https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation/blob/ros2/ur_simulation_gz/doc/migration/lyrical.rst

ur_simulation_gz
^^^^^^^^^^^^^^^^

Remove scaled JTC and change default controller
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The scaled joint torque controller has been removed from ur_controllers in favor of the default
joint trajectory controller, which is now the default one for the GZ simulation as well. This means
that if you were using the scaled JTC before, you will need to change your controllers file to use
the regular JTC instead, and update your MoveIt! configuration accordingly if you were using it
with MoveIt!.
