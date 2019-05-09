cmake_minimum_required(VERSION 2.8.3)

add_compile_options(-std=c++11)

include_directories(${ROS_LIB_DIR})

# Remove this if using an Arduino without native USB
add_definitions(-DUSB_CON)

link_directories(${CMAKE_CURRENT_SOURCE_DIR}/libraries)

generate_arduino_firmware(main
  SRCS firmware.cpp ${ROS_LIB_DIR}/time.cpp
  BOARD mega2560
  #PORT /dev/ttyS0
  PORT /dev/ttyACM0
  #RECURSE True
)
