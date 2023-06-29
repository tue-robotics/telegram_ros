cmake_minimum_required(VERSION 3.0.2)
project(telegram_ros)
find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  std_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS sensor_msgs std_msgs
)

#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()
