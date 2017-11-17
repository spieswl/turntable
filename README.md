## turntable - v1.0.0

*I just could not shake the visual that the 2R robot arm is cranking a turntable, thus, the name stuck.*

The purpose of this ROS package is to evaluate the creation, excitation, and simulation of a simple URDF describing a two-link, two-joint robotic actuator. This package contains two scripts designed to 1) generate joint angles to direct the actuator to follow a circular trajectory and 2) publish markers at points along the path of the actuator. If desired, the package components can be run individually, however, it is **STRONGLY** recommended to use one of the provided launch files to run this package.

At runtime, the launch files will set the robot description on the parameter server, run the required nodes, and launch `rviz` with a given configuration file. Nominally, all nodes and topics needed will be inside the `turntable1` namespace. This namespace can be configured by modifying the [demonstration launch file](https://github.com/spieswl/turntable/blob/master/launch/demo.launch).

A description of the interfaces in use, an overview of the robot "DJ", a quick breakdown of the code, and helpful tips for using this package immediately follow.


### Interfaces
##### Subscribers
- run_turntable.py
  - The node in this script does not subscribe to any topics.

- crank_tracker.py
  - The node in this script does not subscribe to any topics.

##### Publishers
- run_turntable.py
  - This node publishes joint angles for revolute joint 1 (J1) and joint 2 (J2) on the `joint_states` topic in this node's namespace. The data format is of type **[sensor_msgs/JointState](http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html)**.

- crank_tracker.py
  - This node publishes Marker information for spawning markers viewable in `rviz`. The relevant marker data is published on the `path` topic in this node's namespace. The data format is of type **[visualization_msgs/Marker](http://docs.ros.org/api/visualization_msgs/html/msg/Marker.html)**.

##### tf Listeners
- run_turntable.py
  - The node in this script does not listen for any TF data.

- crank_tracker.py
  - The node in this script listens uses `tf2` to listen for transform data from the **world** frame to the **EE** frame. This transform data is of type **[geometry_msgs/TransformStamped](http://docs.ros.org/api/geometry_msgs/html/msg/TransformStamped.html)**.


### Robot Overview

The robot in this package is aptly named "DJ". The URDF for DJ is found [here](https://github.com/spieswl/turntable/blob/master/urdf/DJ.urdf). You can see the `graphviz` representation of DJ's construction by looking at [this Graphviz PDF](https://github.com/spieswl/turntable/blob/master/urdf/DJ.pdf).

Since DJ is fairly simple, his construction can be essentially represented as...

`[base] <--J1--> [L1] <--J2--> [L2] <--EE--> [EE]`

...where `[links]` and `<--joints-->` are represented as shown.


### Code Description

[Run_turntable.py](https://github.com/spieswl/turntable/blob/master/scripts/run_turntable.py) handles publishing joint angle solutions for the arm to follow a circular tracjectory on the aforementioned `joint_states` topic. This script will perform some intial ROS setup, establish the update rate at 50 Hz, and loop through a process which entails 1) determining new joint angles for J1 and J2 by using the included function `IK_2R_2L`, and 2) forming the JointState message using the newly found angles, current system time, and a sequence ID. Note that the frame_ID called out is for the end effector (EE).

This package does not have any specific state publishing mechanism included. Instead, it leverages the `robot_state_publisher` package (*see the included launch files for an example of starting a node for this*) to make `tf` data available to other potential consumers.

[Crank_tracker.py](https://github.com/ME495-EmbeddedSystems/spieswl/turntable/blob/master/scripts/crank_tracker.py) requires that valid `tf` data be available; if so, it will take the relevant pieces of the most recently available transform information to create markers. These markers are published at a rate of 20 Hz at the end of DJ's arm. Much of the code in this file is actually formatting the marker icons that you will see when using `rviz`.

<img src="https://github.com/spieswl/turntable/raw/master/images/results.png" width=480/>


### Helpful Inclusions
##### Launch Files

- To run a demo of the 2R robotic arm in action, enter:
`roslaunch turntable demo.launch`

- To debug the functionality of the 2R robotic arm in `rviz`, enter:
`roslaunch turntable debug_urdf.launch`


### References

None
