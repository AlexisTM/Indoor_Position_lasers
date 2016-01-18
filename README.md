Positionning system with fixed lasers : 
---------------------------------------

Requierments
-----------

Software 
----------

* Working ROS Embedded Linux (RPi2 under Ubuntu 14.04 or Odroid-XU4)
* Mavros & Mavros Extras installed
* Rosserial Installed
* Knowing how to create a ROS workspace

Install
-----------

* Create & source a workspace
* Clone rosserial into the `src` folder of your catkin workspace
* Copy laserpack folder into the `src` folder of your catkin workspace 
* Build your workspace :  `catkin_make` or `catkin build` 
* Build Arduino librairies 
	* rosrun rosserial_Arduino make_librairies.py destination_folder # destination_folder can be `.` for **here** or `~` for **home**. There should be NO folder called ros_lib in the destination_folder
	* copy ros_lib folder to the Arduino librairies
* Start/restart the Arduino IDE, build & upload the code
* Edit the launch file from rosserial to change the Arduino Serial port(`roscd rosserial`, `nano launch/node_serial.launch`) from ttyACM0 to ttyACM1 or ttyUSB (depends on how the Arduino is detected)

CheckList for fast start-up
----------

* Start the Linux embedded
* Connect the PixHawk via a FTDI connector to TELEM1 or USB *NOT RECOMMANDED* (The USB connector is disabled by the PixHawk if there is no connection since 30 seconds (save power))
* `$ roscore` # Start ROS main process
* Connect the Arduino via the USB (Note : On the Odroid, there is a conflict using the Arduino & PixHawk on the same hub... The PixHawk shutdown & restart when the Arduino is removed. No explanation)
* `$ roslaunch mavros px4.launch` # Start Mavros HAL(Hardware Abstraction Layer) service 
* `$ roslaunch rosserial_server node_serial.launch` # Listen to the Arduino, it will publish data to `/lasers/raw`
* `$ rosrun laserpack position_algorithm.py` # Transform measurements to position and euler angles
* `$ rosrun laserpack pixhawk_control.py` # Send positions to the PixHawk, Arm, Offboard, Setpoints

Arduino
----------

* Fritzer schema
* Connect it

Architecture
-----------

* Roscore   : Main node
* Mavros    : MAVLink abstraction layer (MAVLink over UART protocol)
  - Publish to /mavros topics
  - Subscribe to /mavros topics
* Rosserial : Arduino abstraction layer (UART Standardized protocol)
  - rosserial_python/message_definitions.py - gives definitions of the messages
  - rosserial_python/serial_node.py - python uart listener
  - rosserial_server/serial_node - C++ uart listener : Better efficiency
  - Publish to /lasers/raw
* laserpack : Custom messages definitions & script container
  - laserpack/position_algorithm.py - Translate readings to position
  	- Subscribe to /lasers/raw  (laserpack::distance type)
  	- Subscribe to /mavros/imu/data (sensors_msgs::Imu type)
  	- Publish to /lasers/pose (sensors_msgs::PoseStamped type)
  - laserpack/pixhawk_control.py
    - Subscribe to /lasers/pose (sensors_msgs::PoseStamped type) - Gives the custom position we will send
  	- Subscribe to /mavros/imu/data (sensors_msgs::Imu type) - Get the PixHawk IMU
    - Subscribe to /mavros/state (mavros_msgs::State type)
    - Subscribe to /mavros/local_position/pose (sensors_msgs::PoseStamped type)
  	- Service to /mavros/cmd/arming (mavros_msgs::CommandBool type)
    - Service to /mavros/set_mode (mavros_msgs::SetMode type)
 	- Publish to /mavros/setpoint_position/local (sensors_msgs::PoseStamped type)
 	- Publish to /mavros/mocap/pose (sensors_msgs::PoseStamped type)

Todo
-----------

* Adapt Algorithm to get Euler angles from 6 measurments
* Create a roslaunch that start everything



