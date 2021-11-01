# icm20x_imu
Library for using the icm20x Imu, specifically the icm20948

# Installation
These steps should allow you to completely install and run an icm20948 thoruhg a usb to ft232h connection

### Devices needed
* FT232H chip with jst and usb c connectors
* Jst 4-pin cable
* USB A to USB C cable
* Icm20948 9-dof imu

Connect the ft232h to the icm20948 using the jst cable. Then connect the usb c to the ft232h, and plug in to the desired system via USB A

### Libraries
* python3 and ROS (Tested with ROS Noetic)
* Blinka CircuitPython - Use [this page in this guide](https://learn.adafruit.com/circuitpython-on-any-computer-with-ft232h/linux) to install blinka circuit python to your machine
* Install the icm20x library [here](https://github.com/adafruit/Adafruit_CircuitPython_ICM20X). Use the associated example script for the icm20948 to test your connection
* Install this package in a catkin workspace and use imu_test.launch to make sure the imu runs and displays correct values

# Contents
* imu_test.py and imu_test.launch, for running the node and testing initial connection
* icm20948_node.py and imu_node.launch, for actual use in a system
* Also included in the scripts directory is the node_startup.bash script, which can be used to update the ROS_MASTER_URI, ROS_HOSTNAME, and BLINKA_FT232H values needed for the usbl nodes 

