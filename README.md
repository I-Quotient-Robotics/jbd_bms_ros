<<<<<<< HEAD
# jbd_bms_ros
### Battery Management System BMS jbd
## Overview
Read battery-related information returned by BMS，You can configure serial port name, baudrate and loop rate from outside,
Information will be displayed dynamically depending on the number of batteries and temperature sensors，\
You can also get the error bits of information and the error content.
## Equipment Type
Jbd Power Lithium Battery\
![](https://github.com/I-Quotient-Robotics/iqr_jbd_bms/blob/master/type_pic/144283718.jpg)
![](https://github.com/I-Quotient-Robotics/iqr_jbd_bms/blob/master/type_pic/60348685.jpg)

## Environment
Ubuntu16.04\
ROS-kinetic
## Nodes
### jbd_bms_status
jbd_bms_status is a driver for jbd_bms. It reads battery info convert to JbdStatus message.
#### Published Topics
bms(jbd_bms_msg/JbdStatus)\
it publishes bms topic from the jbd_bms.
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
jbd_id(string, default: jbd_bms)\
frame ID for the device.\
## install
```bash
git clone https://github.com/I-Quotient-Robotics/iqr_jbd_bms
cd iqr_jbd_bms
rosdep install jbd_bms_driver --ignore-src
rosdep install jbd_bms_msg
```
## rules
```bash
sudo cp jbd_bms_driver/udev/10-jbd-bms.rules /etc/udev/rules.d/ or /lib/udec/rules.d
sudo udevadm control--reload-rules && udevadm trigger
```
## run
```bash
cd workspace
catkin_make
source /devel/setup.bash
roslaunch jbd_bms_bringup jbd_bms_status.launch
```
## Official website
http://www.jiabaida.com/

=======
# jbd_bms_ros
ROS package for Jiabaida BMS
>>>>>>> 794fe117ccaeb42a8473f843fffd22f51655806e
