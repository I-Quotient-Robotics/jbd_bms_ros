# iqr_hongfu_bms
### Battery Management System BMS
## Overview
Read battery-related information returned by BMS，You can configure serial port name, baudrate and loop rate from outside,
Information will be displayed dynamically depending on the number of batteries and temperature sensors，\
You can also get the error bits of information and the error content.
## Equipment Type
Hongfu Power Lithium Battery\
![](https://github.com/I-Quotient-Robotics/iqr_hongfu_bms/blob/master/type_pic/144283718.jpg)
![](https://github.com/I-Quotient-Robotics/iqr_hongfu_bms/blob/master/type_pic/60348685.jpg)

## Environment
Ubuntu16.04\
ROS-kinetic
## Nodes
### hongfu_bms_status
hongfu_bms_status is a driver for hongfu_bms. It reads battery info convert to HongfuStatus message.
#### Published Topics
bms(hongfu_bms_msg/HongfuStatus)\
it publishes bms topic from the hongfu_bms.
#### Parameters
port_bms(string, default: /dev/port_link_bms)\
serial port name used in your system.\
\
baudrate_bms(int, default: 9600)\
serial port baud rate.\
\
looprate_bms(int, defaule: 2)\
loop rate.\
\
hongfu_id(string, default: hongfu_bms)\
frame ID for the device.\
## install
```bash
git clone https://github.com/I-Quotient-Robotics/iqr_hongfu_bms
cd iqr_hongfu_bms
rosdep install hongfu_bms_driver --ignore-src
rosdep install hongfu_bms_msg
```
## rules
```bash
sudo cp hongfu_bms_driver/udev/10-hongfu-bms.rules /etc/udev/rules.d/ or /lib/udec/rules.d
sudo udevadm control--reload-rules && udevadm trigger
```
## run
```bash
cd workspace
catkin_make
source /devel/setup.bash
roslaunch hongfu_bms_bringup hongfu_bms_status.launch
```


