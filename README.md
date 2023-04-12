# icm20x_imu
Library for using the icm20x Imu, specifically the icm20948

# Installation
These steps should allow you to completely install and run an icm20948 thoruhg a usb to ft232h connection

## Devices needed
* FT232H chip with jst and usb c connectors
* Jst 4-pin cable
* USB A to USB C cable
* Icm20948 9-dof imu

Connect the ft232h to the icm20948 using the jst cable. Then connect the usb c to the ft232h, and plug in to the desired system via USB A

**Note:** if the Ft232h chip is a newer version with a small switch at the top that says 'i2c mode', make sure it is switched to 'on'

## Install Prerequisites   
These intaillation instructions are modified from [here](https://learn.adafruit.com/circuitpython-on-any-computer-with-ft232h/linux).

Tested with python3 and ROS (Tested with ROS Noetic)

**Install libusb**
```
sudo apt-get install libusb-1.0-0-dev
```

**Setup udev rules**

Add the following to `etc/udev/rules.d/11-ftdi.rules`
```
# /etc/udev/rules.d/11-ftdi.rules
SUBSYSTEM=="usb", ATTR{idVendor}=="0403", ATTR{idProduct}=="6001", GROUP="plugdev", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="0403", ATTR{idProduct}=="6011", GROUP="plugdev", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="0403", ATTR{idProduct}=="6010", GROUP="plugdev", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="0403", ATTR{idProduct}=="6014", GROUP="plugdev", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="0403", ATTR{idProduct}=="6015", GROUP="plugdev", MODE="0666"
```

**Install pyftdi**

The pyftdi library has been changed to decrease latency and need to be installed from source. The original code can be found [here](https://github.com/eblot/pyftdi). 

```bash
git clone git@github.com:AutonomousFieldRoboticsLab/pyftdi.git
cd pyftdi
pip  install .
```

**Install Adafruit-Blinka**
```bash
pip install Adafruit-Blinka
```

**Install the icm20x library**
The original library is located [here](https://github.com/adafruit/Adafruit_CircuitPython_ICM20X). This library has been changed to remove some sleep commands. 

```bash
git clone git@github.com:AutonomousFieldRoboticsLab/Adafruit_CircuitPython_ICM20X.git
cd Adafruit_CircuitPython_ICM20X
pip install .
```

There are scripts inside examples folder to test icm20948 connection. You can test them.

### Installation

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/AutonomousFieldRoboticsLab/icm20x_imu
cd ~/catkin_ws
catkin_make
```

### Usage

```bash
cd ~/catkin_ws
source devel/setup.bash
roslaunch icm20x_imu imu_node.launch 
```

**Note**
The imu rate is set to 80. The magentic field rate is set to 20Hz (imu rate / 4). The speed could not be increased more than this. 
* Decreasing the latency in the pyftdi library to 1ms results in abrupt connection loss.


