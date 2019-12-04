# Robot Simulation with V-REP

Repository for ELEC4010K course project, Fall 2019. This is a simulation of a differential drive robot with ROS and the V-REP simulator, performing the following tasks:

* Mapping with simulated Lidar data using Hector SLAM
* Face recognition and localization
* Visual servoing

### Robot control in V-REP

Robot control is performed using the [key_teleop](http://wiki.ros.org/key_teleop) node. Install the node by

```
sudo apt install ros-melodic-key-teleop
```

After installation, we need to change the name of the published control messages. In the file `<path-to-ros>/lib/key_teleop/key_teleop.py`, look for the line `self._pub_cmd = rospy.Publisher('key_vel', Twist)` and replace `key_vel` with `/vrep/cmd_vel`.

The node should now be set up. Start the simulation in V-REP and run the node in a new terminal:

```
rosrun key_teleop key_teleop.py
```

Now control the robot with arrow keys while focused on the `key_teleop` terminal window. The robot should move.
