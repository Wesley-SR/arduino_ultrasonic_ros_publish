------------------------------------------------------------------------------------------
ULTRASOUND PUBLISHER WITH MEGA ARDUINO
------------------------------------------------------------------------------------------
 - This repository aims to utility six ultrasonic sensors in a Arduino.
 - For ROS, use the ROS_publisher folder.
 - To read the measurements only on the Arduino serial, use the ultrasound_serial_monitor folder.

Sources:
1) https://www.intorobotics.com/how-to-use-sensor_msgs-range-ros-for-multiple-sensors-with-rosserial/
2) https://github.com/surabhi96/Library-navigating-robot/wiki/Ultrasonic-sensor-with-ROS 



------------------------------------------------------------------------------------------
STEP BY STEP TO USE WITH ROS:
------------------------------------------------------------------------------------------

1 - Load the ROS_publisher code on the arduino


2 - Run ROSCORE on a terminal

  OBS: For each new open terminal run
    $ cd ~/catkin_ws/
    $ source devel/setup.bash
  
  Open ROSCORE
  $ roscore


3 - Run this command (in a new terminal) to find out which serial ports the Arduino is connected to

  % ls /dev/ttyACM* 
  or
  % ls /dev/ttyUSB*


4 - Run the rosserial_python fot the respective serial port, for example,

  $ rosrun rosserial_python serial_node.py /dev/ttyACM0


5 - List active topics

  $ rostopic list


6 - Run rostopic to view what's being posted on ROS topics

  $ rostopic echo ultrasonic_1 (New terminal)
  $ rostopic echo ultrasonic_2 (New terminal)
  $ rostopic echo ultrasonic_3 (New terminal)
  $ rostopic echo ultrasonic_4 (New terminal)
  $ rostopic echo ultrasonic_5 (New terminal)
  $ rostopic echo ultrasonic_6 (New terminal)


7 - If you want to save the data in bagfile

  $ cd rosbag ...
  $ rosbag record -a


8 - If you want to convert rosbag to csv
  $ rosrun rosbag_to_csv rosbag_to_csv.py




------------------------------------------------------------------------------------------
STEP BY STEP TO USE WITHOUT ROS:
------------------------------------------------------------------------------------------

1 - Load the ultrasound_serial_monitor code on the Arduino and view the measurements on the serial monitor



------------------------------------------------------------------------------------------
  PINS
------------------------------------------------------------------------------------------

 --- TRIGGER PINS ---
 
 - Sensor 1 (Board) - Pin 04 (Arduino) - FE (Robot)
 - Sensor 2 (Board) - Pin 05 (Arduino) - FC (Robot)
 - Sensor 3 (Board) - Pin 06 (Arduino) - FD (Robot)
 - Sensor 4 (Board) - Pin 07 (Arduino) - TD (Robot)
 - Sensor 5 (Board) - Pin 08 (Arduino) - TC (Robot)
 - Sensor 6 (Board) - Pin 09 (Arduino) - TE (Robot)


 --- ECHO PINS ---
 
 - Sensor 1 (Board) - Pin 02 (Arduino) - FE (Robot)
 - Sensor 2 (Board) - Pin 03 (Arduino) - FC (Robot)
 - Sensor 3 (Board) - Pin 18 (Arduino) - FD (Robot)
 - Sensor 4 (Board) - Pin 19 (Arduino) - TD (Robot)
 - Sensor 5 (Board) - Pin 20 (Arduino) - TC (Robot)
 - Sensor 6 (Board) - Pin 21 (Arduino) - TE (Robot)

