iiwa_toolkit
========
 

Requirements
-----------

iiwa_toolkit requires several packages to be installed in order to work properly:

* [ROS] - ROS: tested in **Melodic** and **Kinetic**; *Indigo* should work also
* [iiwa_ros] 
* [passive_ds_control] 
* [Util]


Compilation
------------

```sh
cd /path/to/ros_workspace
# source ros workspace
catkin_make
```

Basic Usage
--------------

### Bringup iiwa_driver

**To launch passive tracker either in position or orientation or velocity**
```sh
roslaunch iiwa_toolkit passive_track_real.launch
```

### Gazebo Simulation

**To launch passive tracker either in position or orientation or velocity**
```sh
roslaunch iiwa_toolkit passive_track_gazebo.launch
```

Both of the above commands will launch IIWA in **torque-control mode**. To change the control mode (e.g., position-control), please edit the launch files to select the appropriate controller.


To do 
--------------
- removing the dependency on passive_ds_control and Util
- add launcher with rqt plot
Documentation
---------------------

**UNDER CONSTRUCTION**

Contributing
---------------------
TBW ...

Acknowledgements
---------------------
TBW ...

Authors/Maintainers
---------------------
- Farshad Khadivar (farshad.khadivar@epfl.ch)
