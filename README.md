# REACH ROS Plugins
![](https://img.shields.io/badge/License-Apache%202.0-blue.svg)
[![Ubuntu](https://github.com/ros-industrial/reach_ros2/actions/workflows/ubuntu.yml/badge.svg)](https://github.com/ros-industrial/reach_ros2/actions/workflows/ubuntu.yml)

This package contains the ROS2-based plugin implemenations of REACH kinematics, evaluation, and display interfaces

![REACH ROS](demo/docs/reach_study_demo.gif)

## Installation
First, clone the repository into a `colcon` workspace
``` bash
cd ~/reach_ws/src
git clone https://github.com/ros-industrial/reach_ros2.git
cd ..
```

Install the dependencies
``` bash
vcs import src < src/reach_ros2/dependencies.repos
rosdep install --from-paths src --ignore-src -r -y
```

Build the repository
```
colcon build --symlink-install
```

## Demo
A simple demonstration of the capability of this repository is provided in the `demo` sub-directory.
See the [instructions](demo/README.md) for details on how to launch the demo from ROS 2 and see the [Python instructions](demo/python/README.md) to see how you can use the Python interface to launch studies.

## Usage
Use the following steps to run a reach study with a robot using the ROS1 infrastructure and plugins.

1. Create any files describing your robot system required by the REACH plugins (e.g., URDF, SRDF, kinematics file, joint limits file, etc.)
1. Create a mesh model of the workpiece
    > Note: the origin of this model should align with the kinematic base frame of the robot
1. Create a point cloud of the target points on the workpiece
    - This point cloud can be generated using a command line tool from PCL 1.8:
      ```
      pcl_mesh_sampling <workpiece_mesh>.ply <output_cloud>.pcd -n_samples <number of samples> -leaf_size <leaf_size> -write_normals true
      ```
1. Create a configuration YAML file defining the parameters of the reach study and the configuration of the interface plugins (see [this demo example](demo/config/reach_study.yaml))
1. Run the setup launch file
    ```
    ros2 launch reach_ros setup.launch robot_description_file:=<path_to_URDF>
    ```
1. Run the reach study analysis
    ```
    ros2 launch reach_ros start.launch \
      robot_description_file:=<path_to_URDF> \
      robot_description_semantic_file:=<path_to_SRDF> \
      robot_description_kinematics_file:=<path_to_kinematics.yaml> \
      robot_description_joint_limits_file:=<path_to_joint_limits.yaml> \
      config_file:=<config_file.yaml> \
      config_name:=<arbitrary_config> \
      results_dir:=<arbitrary_results_directory>
    ```

## Evaluation Plugins

### Manipulability

This plugin uses MoveIt! to calculate the manipulability of a robot pose. Higher manipulability results in a higher pose score. Range: [0, inf)

Parameters:

- **`planning_group`**
  - The name of the planning group with which to evaluate the manipulability of a given robot pose
- **`jacobian_row_subset`** (optional)
  - The indices of the rows of the Jacobian to use when evaluating the manipulability. The row indices should be on [0, 6) and correspond to the output space [x, y, z, rx, ry, rz]
  - Ex. `jacobian_row_subset: [0, 1, 2]  # position manipulability only`

### Manipulability Scaled

This plugin uses MoveIt! to calculate the manipulability of a robot pose divided by the characteristic length of the motion group.
The characteristic length is computed by walking from the base link to the tip link of the motion group and summing the distances between adjacent links.
Higher scaled manipulability results in a higher pose score. Range: [0, inf)

Parameters:

- **`planning_group`**
  - The name of the planning group with which to evaluate the manipulability of a given robot pose
- **`jacobian_row_subset`** (optional)
  - The indices of the rows of the Jacobian to use when evaluating the manipulability. The row indices should be on [0, 6) and correspond to the output space [x, y, z, rx, ry, rz]
  - Ex. `jacobian_row_subset: [0, 1, 2]  # position manipulability only`
- **`excluded_links`** (optional)
  - The names of links contained in the motion group that should not contribute to the characteristic length

### Manipulability Ratio

This plugin uses MoveIt! to calculate the manipulability of a robot pose and evaluate a score. The score is calculated as the ratio of the smallest manipulability value to the largest manipulability value.
The larger this ratio, the more uniform the dexterity and the higher the score. Range [0, 1]

Parameters:

- **`planning_group`**
  - The name of the planning group with which to evaluate the manipulability of a given robot pose
- **`jacobian_row_subset`** (optional)
  - The indices of the rows of the Jacobian to use when evaluating the manipulability. The row indices should be on [0, 6) and correspond to the output space [x, y, z, rx, ry, rz]
  - Ex. `jacobian_row_subset: [0, 1, 2]  # position manipulability only`

### Distance Penalty

This plugin uses the MoveIt! collision environment to calculate the distance to closest collision
for a robot pose. That distance value is then used to score the robot pose. Larger distance to closest collision
results in higher pose score. Range: [0, 1]

Parameters:

- **`planning_group`**
  - The name of the planning group to be used to solve the robot's inverse kinematics
- **`distance_threshold`**
  - The distance between 2 closest surfaces to collision under which an inverse kinematics solution will be considered invalid
- **`collision_mesh_filename`** (optional)
  - The filename (in ROS package URI format) of the reach object mesh to be used to do collision checking
  - Example: `package://<your_package>/<folder>/<filename>.stl
- **`collision_mesh_frame`** (optional)
  - The TF frame to which the collision mesh should be attached
  - If left unspecified, the collision mesh will be attached to the kinematic base frame associated with `planning_group`
- **`touch_links`**
  - The names of the robot links with which the reach object mesh is allowed to collide
- **`exponent`**
  - score = min(abs(closest_distance_to_collision / distance_threshold), 1.0)^exponent.

### Joint Penalty

This plugin uses the MoveIt! robot model to calculate a robot pose score based on how much the pose deviates
from the center of the joint range. Robot poses that are closer to the center of the joint range result in higher pose scores. Range: [0, 1]

Parameters:

- **`planning_group`**
  - The name of the planning_group with which to evaluate the joint penalty

## IK Solvers

### MoveIt! IK Solver

This plugin uses MoveIt! kinematics solvers and collision checking to produce collision aware IK solutions

Parameters:

- **`planning_group`**
  - Name of the planning group
- **`distance_threshold`**
  - The distance from nearest collision at which to invalidate an IK solution. For example, if this parameter is
  set to 0.1m, then IK solutions whose distance to nearest collision is less than 0.1m will be invalidated
- **`collision_mesh_filename`** (optional)
  - The file path to the collision mesh model of the workpiece, in the `package://` or 'file://' URI format
- **`collision_mesh_frame`** (optional)
  - The TF frame to which the collision mesh should be attached
  - If left unspecified, the collision mesh will be attached to the kinematic base frame associated with `planning_group`
- **`touch_links`**
  - The TF links that are allowed to be in contact with the collision mesh
- **`evaluation_plugin`**
  - The name (and parameters) of the evaluation plugin to be used to score IK solution poses

### Discretized MoveIt! IK Solver

This plugin performs the same function as the MoveIt! IK solver plugin above, but calculates IK solutions for
a target that has been discretized about its Z-axis by an input angle. The pose with the best score is returned.

Parameters:

- **`planning_group`**
  - Name of the planning group
- **`distance_threshold`**
  - The distance from nearest collision at which to invalidate an IK solution. For example, if this parameter is
  set to 0.1m, then IK solutions whose distance to nearest collision is less than 0.1m will be invalidated
- **`collision_mesh_filename`** (optional)
  - The file path to the collision mesh model of the workpiece, in the `package://` or 'file://' URI format
- **`collision_mesh_frame`** (optional)
  - The TF frame to which the collision mesh should be attached
  - If left unspecified, the collision mesh will be attached to the kinematic base frame associated with `planning_group`
- **`touch_links`**
  - The TF links that are allowed to be in contact with the collision mesh
- **`evaluation_plugin`**
  - The name (and parameters) of the evaluation plugin to be used to score IK solution poses
- **`discretization_angle`**
  - The angle (between 0 and pi, in radians) with which to sample each target pose about the Z-axis

## Display Plugins

### ROS Reach Display

This plugin publishes joint state and visualization markers to display the results of the reach study.
The markers are interactive:
  - Left-clicking on a marker will change the robot position to the IK solution
    - This works by publishing a `sensor_msgs/JointState` message to the `reach_joints` topic, which is a source topic for the `joint_state_publisher`
  - Right-clicking on the marker will bring up a context menu that displays the score of the reach target

The markers are colorized with a heat-map, where red represents the highest scores (i.e., hottest) and blue represents the lowest scores (i.e., coldest).
Unreachable points are colorized black. There are two methods for computing the values for the heat map:
  1. By default, the values for the heat map are computed as the ratio of the individual scores to the maximum score.
  Thus if the scores of the reach study are all fairly consistent and close to the maximum, then the colors of the markers will be mostly red (with the target with the maximum score being the deepest hue of red).
  If the scores of the reach study targets differ greatly, more variation in color will be seen.
  Usually this method produces the most meaningful display.
  1. The values can also be computed using the "full color range", where the scores are fully normalized before colorization.
  The lowest score (regardless of value) is always displayed as the deepest hue of blue and the the highest score is always shown as the deepest hue of red.
  This is valuable for highlighting differences in reachability but can be misleading due to the normalization of the scores.

Parameters:

- **`collision_mesh_filename`**
  - The file path to the collision mesh model of the workpiece, in the `package://` or `file://` URI format
- **`collision_mesh_frame`**
  - The TF frame to which the collision mesh should be attached
  - If left unspecified, the collision mesh will be attached to `kinematic_base_frame`
- **`kinematic_base_frame`**
  - The base frame of the kinematic tree in which to display the interactive markers
- **`marker_scale`**
  - The length (in meters) of the arrow markers representing the target Cartesian points
- **`use_full_color_range`** (optional, default: False)
  - Colorize the heat map using the full range of colors (such that the target with the lowest score is the deepest hue of blue, and the target with the highest score is the deepest hue of red)
- **`hue_low_score`** (optional, default: 270.0)
  - Allows changing the default heatmap style coloring to a different color scale in HSV space (e.g. `hue_low_score: 0.0, hue_high_score: 180.0` will lead to a red to green color scale).
- **`hue_high_score`** (optional, default: 0.0)
  - See `hue_low_score` for an explanation.

## Target Pose Generator Plugins

### Transformed Point Cloud Target Pose Generator

This plugin inherits `reach::PointCloudTargetPoseGenerator`, which creates target reach study waypoints from a point cloud file with normals, and transforms the point cloud points into a desired URDF frame (typically the kinematic base frame of the robot).

Parameters:

- **`pcd_file`**
  - The path to the point cloud file (.pcd format), in the `package://` or `file://` URI
- **`points_frame`**
  - The frame of the URDF to which the point cloud points are relative (i.e. the origin frame of the point cloud)
- **`target_frame`**
  - The frame into which the point cloud points should be transformed for the reach study purposes. This should be the kinematic base frame of the robot
