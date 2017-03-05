cmake_minimum_required(VERSION 2.8.3)
project(vrs_gps_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs
  message_generation
)

add_message_files(
  FILES
  vrs_gps.msg
)

catkin_package(
  CATKIN_DEPENDS
    serial
    std_msgs
    message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(vrs_gps_driver src/vrs_gps_driver.cpp)

target_link_libraries(vrs_gps_driver
  ${catkin_LIBRARIES}
)

