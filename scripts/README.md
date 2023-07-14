# REACH ROS Python

This package also provides a Python interface that allows executing the ROS 2 based plugins from Python.

REACH uses a ROS node singleton to allow all plugins to share the same node.
For this to work, it is necessary to initialize ROS and to provide the parameters to the node.
Since the node is written in C++, the `rclpy` methods can not be used.
Instead, the Python interface provides methods to initialize ROS with any given command line arguments and to manually handle the parameters of the node.

## Python reach study node

The [`reach_ros_node.py`](./reach_ros_node.py) script is a Python equivalent of the [c++ reach study node](../src/reach_study_node.cpp).
To test it out, first modify the name of the executable launched in [`start.launch.py`](../launch/start.launch.py):

```diff
Node(
    package='reach_ros',
-   executable='reach_ros_node',
+   executable='reach_ros_node.py',
    output='screen',
```

 Then run the demo following the instructions [here](../demo/README.md).

## Python reach study demo

The [`demo.py` script](demo.py) will launch the same demo (described [here](../README.md)) but from Python.
It will run the reach study two times, once with the original MoveIt IK solver parameters and once with a shorter timeout.
The results are then compared.
Please, also have a look into the Python code to see how this works.

You can start the demo like this:
```
ros2 run reach_ros demo.py
```

