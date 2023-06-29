# REACH Python Demo

This package also provides a Python interface that allows executing the ROS 2 based plugins from Python.

REACH uses a ROS node singleton to allow all plugins to share the same node.
For this to work, it is necessary to initialize ROS and to provide the parameters to the node.
Since the node is written in C++, the `rclpy` methods can not be used.
Instead, the Python interface provides methods to initialize ROS with any given commandline arguments and to manually handle the parameters of the node.

## Launching the Demo from Python

This will launch the same demo (described [here](../README.md)) from Python.
It will run it two times. Once with the original MoveIt IK solver parameters and once with a lower timeout.
The results are then compared.
Please, also have a look into the Python code to see how this works.

You can start the demo like this:
```
ros2 run reach_ros demo.py
```

