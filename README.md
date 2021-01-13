# arduino_ultrasonic_ros_publish
This repository aims to utility multiple ultrasonic sensors in a Arduino and publish the ranges in a ROS topic.

Sources:
1) https://www.intorobotics.com/how-to-use-sensor_msgs-range-ros-for-multiple-sensors-with-rosserial/
2) https://github.com/surabhi96/Library-navigating-robot/wiki/Ultrasonic-sensor-with-ROS

STEP BY STEP TO USE:
1 - Upload the code to your arduino


2 - Run ROSCORE on a terminal

  OBS: For each new open terminal run
    $ cd ~/catkin_ws/
    $ source devel/setup.bash
  
  Open ROSCODE
  $ ROSCORE


3 - Run this command to find out which of the serial ports the Arduino is connected to
  % ls /dev/ttyACM* (New terminal)


4 - Run the rosserial_python
  $ rosrun rosserial_python serial_node.py /dev/ttyUSB0


5 - List active topics
  % rostopic list


6 - Run rostopic to view what's being posted on ROS topics
  % rostopic echo ultrasonic_1 (New terminal)
  % rostopic echo ultrasonic_2 (New terminal)
  % rostopic echo ultrasonic_3 (New terminal)
  ...
