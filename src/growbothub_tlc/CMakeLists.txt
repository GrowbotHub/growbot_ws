cmake_minimum_required(VERSION 2.8.3)
project(growbothub_tlc)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

add_service_files(
  FILES
  DeviceReadWrite.srv
  DeviceSummary.srv
)

generate_messages(
  DEPENDENCIES 
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
)
#add_executables(${growbothub_tlc}_node src/I2C.py)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
