# Robot Simulation with V-REP

Repository for ELEC4010K course project, Fall 2019. This is a simulation of a differential drive robot with ROS and the V-REP simulator, performing the following tasks:

* Mapping with simulated Lidar data using Hector SLAM
* Face detection and localization
* Visual servoing

### Dependencies

The program is developed and tested on the following environment:

* Ubuntu 18.04
* ROS *Melodic Morenia*
* V-REP PRO EDU (3.6.2) with ROS Interface
* Python dependencies - install with `pip`
  * NumPy
  * OpenCV (3.2.0)
  * `simple-pid` for PID control
* ROS pacakges / dependencies - install by `sudo apt install ros-melodic-<package name>`
  * [Hector SLAM](http://wiki.ros.org/hector_slam)
  * [`cv_bridge`](http://wiki.ros.org/cv_bridge)
  * [`key_teleop`](http://wiki.ros.org/key_teleop)
  
### Installation

1. Clone repository.
2. Copy the `ball_tracker` and `robot_vision` folders to `<path to catkin_ws>/src`.
3. Source environment: `source <path to ros>/setup.bash`
4. Build with `catkin_make`
5. Overwrite `<path to ros>/lib/key_teleop/key_teleop.py` with the provided file `key_teleop/scripts/key_teleop.py`. Ensure the copied file has execute permissions.

### How to run program

1. Source workspace setup file: `source <path to catkin_ws>/devel/setup.bash`
2. Start `roscore` and V-REP
3. Begin V-REP simulation
4. Head to project directory and launch the main launch file

```
cd <path of this project>
roslaunch launch/main.launch
```

The main launch file will simultaneously launch the following nodes:

* `hector_mapping` for online map creation
* `ball_tracking` for visual servoing on the yellow ball
* `robot_vision` for face detection
* RViz for visualizing the generated map and image marker

5. Launch the control node with sepearate terminal window

```
rosrun key_teleop key_teleop.py
```

Now control the robot with <kbd>&#8593;</kbd><kbd>&#8595;</kbd><kbd>&#8592;</kbd><kbd>&#8594;</kbd> while focused on the `key_teleop` terminal window. You can also press <kbd>a</kbd> to toggle between manual and auto control. During auto control mode, the `key_teleop` node stops controlling the robot, which allow the `ball_tracker` to automatically drive the robot if the yellow ball in sight.

### Warning

* Please switch to auto-driving mode *only* when robot is in the room with the yellow ball.
* Laser scan messages are not published during auto-driving mode.

