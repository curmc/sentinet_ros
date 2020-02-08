cmake_minimum_required(VERSION 3.5)
project(kermit)

# Used for compile database 
# This can be unset and it wont change anything
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-std=c++17)


# ROS PACKAGES
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  sensor_msgs
  roscpp
  rospy
  message_generation
)

# EXTERNAL PACKAGES
find_package(OpenCV REQUIRED)
# find_library(APRIL_TAG apriltag)
# find_library(WIIMOTE wiiuse)


# Run the python setup script
catkin_python_setup()


# MESSAGES
add_message_files(
  FILES
  path_state.msg
  mars_imu.msg
  )

# SERVICES
add_service_files(
  FILES
  ping_message.srv
  )

# CATKIN OPTIONS
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)
catkin_package(
  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs geometry_msgs
  )


# C++ Directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


# Kermit Library
# C 
add_library(kermitc
  src/c/kernel/serial.c
  src/c/kernel/teensy_msg.c
  src/c/kernel/teensy_serial.c)


# C++
add_library(kermitcpp
  src/cpp/kernel/Kernel.cpp
  src/cpp/math/smath.cpp)


# Nodes
add_executable(kernel nodes/kernel.cpp)
target_link_libraries(kernel ${catkin_LIBRARIES} kermitc kermitcpp)


# INSTALLATION INSTRUCTIONS
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp")

# install(PROGRAMS nodes/python/controller.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
# install(DIRECTORY src/controller/controller DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

# install(TARGETS kernel_simul camera_simul
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
#
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
