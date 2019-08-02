cmake_minimum_required(VERSION 2.8.3)
project(jbd_bms_msg)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  roscpp
  rospy
  std_msgs
)


add_message_files(
   FILES
   JbdStatus.msg
 )

generate_messages(
  DEPENDENCIES
  std_msgs  
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES jbd_bms_msg
#  CATKIN_DEPENDS message_generation message_runtime roscpp rospy std_msg
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)