cmake_minimum_required(VERSION 3.0.2)
project(telegram_ros)
find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  sensor_msgs
  std_msgs
)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  LabeledImage.msg
  Options.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime sensor_msgs std_msgs
)
