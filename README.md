# WYCA Intership Test - Filter for a Laser Scan, using ROS

A ROS node which subscribes to a LaserScan message and publish the filtered LaserScan.
The filter may be average or median.
The filtering window may be 4 or 8.

## Prerequisites
- ROS (Kinetic or above)
- A Laser Scan as a rosbag file
- CMake

## Launch
###Warning ! 
Before launching the package, you may change the path to the bag file you want to filter into the launch file.
The launch file is located at the root of the package: wyca_internship_test > ProjectLauncher.launch
You may open this file, and change the path to the bag file into your own, line 6:
"<node pkg="rosbag" type="play" name="rosbag" args="/home/anaelle/ros_ws/src/wyca_internship_test/src/2019-11-28-14-30-16.bag"/>"

To launch the package, you may write into a terminal:

```
roslaunch wyca_internship_test ProjectLauncher.launch
```


By default, the filter type is set to average, and the rolling window size is set to 4 elements.
You can change it by specifying the arguments when you launch the package.
Examples:
```
roslaunch wyca_internship_test ProjectLauncher.launch "size":=8
```
```
roslaunch wyca_internship_test ProjectLauncher.launch "type":='median' "size":=8
```

## Built with
* ROS Kinetic
* Ubuntu 16
* QT Creator

## Author
* Anaelle Sarazin-Wronka _alias_ [ana-sw](https://github.com/ana-sw)

