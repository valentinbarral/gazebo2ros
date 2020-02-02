**NOTE:** This repository is related with the next scientific work:

**Barral, V.**; **Escudero, C.J.**; **García-Naya, J.A.**; **Maneiro-Catoira, R.** *NLOS Identification and Mitigation Using Low-Cost UWB Devices.* Sensors 2019, 19, 3464.[https://doi.org/10.3390/s19163464](https://doi.org/10.3390/s19163464)

If you use this code for your scientific activities, a citation is appreciated.

# README

This project contains several ROS nodes:

*  ```gazebo2ros``` Subscribes to Gazebo topics and re-publish them inside ROS.
*  ```gazeboGuide``` Using a joystick, allows the user start and stop the movement of a Gazebo model along a path. 
*  ```gazeboAutoguide``` Moves automatically a Gazebo model along a path. 
*  ```gazeboRestartService``` Relaunches a Gazebo simulation and start moving a model along a path.  

## Building the nodes

To build the nodes, source code must be cloned inside a catkin work space on a ROS installation (see [Creating a workspace for catkin](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)). If the catkin work space is located at ```~/catkin_ws``` then:

```bash
$ cd ~/catkin_ws/src
$ mkdir gtec
$ cd gtec
$ git clone gazebo2ros
```

Then ```catkin_make``` must be used to build the nodes:

```bash
$ cd ~/catkin_ws
$ catkin_make
```

## Requirements

Libraries ```libgazebo9-dev``` ,```ros-XXX-mavros-msgs``` (where XXX is your ROS distro name e.g. "melodic"), and ```libprotoc``` must be installed before building this package. 

Also, package ```gtec_msgs``` must be present in the same workspace. This package can be found here:  [https://github.com/valentinbarral/rosmsgs](https://github.com/valentinbarral/rosmsgs)


## gazebo2ros node

This node subscribes to Gazebo topics and re-publish them inside ROS. Topics published are:

* ```/gtec/gazebo/px4flow``` : Data from a simulated Px4Flow sensor.
* ```/gtec/gazebo/erle/imu``` : Data from an IMU.
* ```/gtec/gazebo/erle/mag``` : Data from a magnetometer.


### Launch

To launch the node, there is a ```.launch``` file that can be used: 

```bash
$ roslaunch gtec_gazebo2ros gtec_gazebo2ros.launch
```

## gazeboGuide node

This node uses a joystick to start the movement of a Gazebo model along a path. The launcher file contains several parameters that must be set to work properly.

```xml
<launch>
    <node name="joystick" pkg="joy" type="joy_node">
        <param name="dev" value="/dev/input/js0">
        </param>
    </node>
    <node name="gazebo_guide" output="screen" pkg="gtec_gazebo2ros" type="gazebo_guide">
        <arg default="7" name="start_button">
        </arg>
        <param name="start_button" value="$(arg start_button)">
        </param>
        <param name="route_file" textfile="$(find gtec_gazebo2ros)/src/Routes/r01.xml">
        </param>
    </node>
</launch>
```
* ```dev```: address of the device where the joystick is attached.
* ```start_button```: index of the button used to start the simulation.
* ```route_file```: path to an ```.xml``` file with the route definition.

### Route definition

To define a route, a ```.xml``` file must be created. The file must follow the next example:

```xml
<route>
    <set x="14.12" y="12.81" angle="270.0"/>
    <moveto x="14.12" y="9.58" time="6.0"/>
    <moveto x="14.12" y="4.87" time="5.0"/>
    <moveto x="14.12" y="4.20" time="1.0"/>
    <stop time="4.0"/>
    <moveto x="14.12" y="0" time="7.0"/>
    <moveto x="16.8" y="-1.63" time="6.0"/>
    <moveto x="18.44" y="0.0" time="4.0"/>
    <moveto x="21" y="0.0" time="6.0"/>
    <spin angle="180.0" time="6.0"/>
    <moveto x="18.44" y="0.0" time="4.0"/>
    <moveto x="16.8" y="-1.63" time="6.0"/>
    <moveto x="14.12" y="0" time="6.0"/>
    <moveto x="14.12" y="4.20" time="6.0"/>
    <stop time="2.0"/>
    <moveto x="14.12" y="4.87" time="2.0"/>
    <moveto x="14.12" y="9.58" time="7.0"/>
    <moveto x="14.12" y="12.81" time="5.0"/>
    <spin angle="180.0" time="6.0"/>
</route>
```
The available commands are:

* ```set``` : Sets a position and orientation.
* ```moveto``` : Moves the vehicle from its current position to the position given in the command. The vehicle will be moved at constant velocity to achieve destination after the number of seconds set under the parameter ```time```.
*  ```stop``` : Stops the vehicle in its current position for an specified number of seconds.
*  ```spin``` : Spins the vehicle the given angle at a constant angular velocity so the vehicle has the final orientation after the number of second specified under the parameter time. 

### Launch

To launch the node, there is a ```.launch``` file that can be used: 

```bash
$ roslaunch gtec_gazebo2ros gazeboGuide.launch
```

## gazeboAutoguide node

This node starts a movement along a path automatically. The launcher file contains several parameters that must be set to work properly.

```xml
<launch>
    <node name="gazebo_auto_guide" output="screen" pkg="gtec_gazebo2ros" type="gazebo_auto_guide">
        <arg default="4.0" name="start_time">
        </arg>
        <param name="start_time" value="$(arg start_time)">
        </param>
        <param name="route_file" textfile="$(find gtec_gazebo2ros)/src/Routes/r01.xml">
        </param>
        <param name="max_reps" value="50"/>
        <param name="record_log" value="1"/>
         <param name="restart_on_finish" value="1"/>
    </node>
</launch>
```
* ```start_time```: movement will start after this amount of seconds.
* ```route_file```: path to an ```.xml``` file with the route definition.
* ```max_reps```: number of repetitions.
* ```record_log```: launches a instance of ```rosbag record``` to record a ROS bag with the data coming from the sensors.
* ```restart_on_finish```: restart the simulation when the vehicle reaches the end of the path.

### Launch

To launch the node, there is a ```.launch``` file that can be used: 

```bash
$ roslaunch gtec_gazebo2ros gazeboAutoGuide.launch
```

## gazeboRestartService node

This node restarts a Gazebo simulation generating a random set of magnetic interferences each time. The launcher file contains several parameters that must be set to work properly.

```xml
<launch>
    <node name="gazebo_restart_service" output="screen" pkg="gtec_gazebo2ros" type="gazebo_restart_service">
         <param name="min_random_x" value="0.0" />
        <param name="max_random_x" value="10.0" />
        <param name="min_random_y" value="0.0" />
        <param name="max_random_y" value="10.0" />
        <param name="min_random_z" value="0.0" />
        <param name="max_random_z" value="10.0" />
    </node>
</launch>
```
* ```min_random_x ```: min X value of a magnetic interference.
* ```max_random_x ```: max X value of a magnetic interference.
* ```min_random_y ```: min Y value of a magnetic interference.
* ```max_random_y ```: max Y value of a magnetic interference.
* ```min_random_z ```: min Z value of a magnetic interference.
* ```max_random_z ```: max Z value of a magnetic interference.

### Launch

To launch the node, there is a ```.launch``` file that can be used: 

```bash
$ roslaunch gtec_gazebo2ros gazeboRestartService.launch
```
