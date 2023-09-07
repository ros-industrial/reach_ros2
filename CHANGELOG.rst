^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package reach_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.3.1 (2023-09-07)
------------------
* Updated python module install (`#22 <https://github.com/marip8/reach_ros2/issues/22>`_)
* Contributors: Michael Ripperger

1.3.0 (2023-08-16)
------------------
* Add customizable color scale for visualization (`#18 <https://github.com/marip8/reach_ros2/issues/18>`_)
* Contributors: Marc Bestmann

1.2.0 (2023-08-16)
------------------
* Added Python equivalent of the c++ reach study node
* Added a Python interface to allow usage of ROS2 plugins in Python
* Revised ROS node singleton pattern
* Remove the plugin library from ament_target_export so downstream packages cannot link against it
* Link executable against main library instead of plugin library
* Fixed typo in plugin export
* Contributors: Marc Bestmann, Michael Ripperger

1.1.0 (2023-06-13)
------------------
* Updated to REACH 1.3.0
* Clean up config/launch files
* Updated demo results
* Updated distance penalty to cap at 1.0
* Updated collision distance evaluator to optionally accept collision mesh; updated README
* Added comments about collision_mesh_frame parameter
* Fix deps install in README
* Updated to reach 1.2.0
* Updated README
* Added transformed point cloud target pose generator
* Exposed interface implementations separately from plugin library
* Fixed collision mesh frame key
* Fixed docker commit name in CI configuration
* Updated node to join executor thread properly
* Added reach plugin library name to environment variable for discovery by the plugin loader
* Updated to reach 1.1.0
* Updated to reach 0.1.1
* Merge pull request `#2 <https://github.com/marip8/reach_ros2/issues/2>`_ from marip8/ros2
  ROS2 Support
* Contributors: Marc Bestmann, Michael Ripperger

1.0.0 (2023-05-10)
------------------
* Move ROS1 components (`#44 <https://github.com/marip8/reach/issues/44>`_)
* Contributors: Michael Ripperger
