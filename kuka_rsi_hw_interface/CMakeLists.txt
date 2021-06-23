cmake_minimum_required(VERSION 2.8.3)
project(kuka_rsi_hw_interface)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  controller_manager
  hardware_interface
  joint_limits_interface
  realtime_tools
  roscpp
  std_msgs
  message_generation
  trajectory_msgs
  sensor_msgs
  fdcc
)



find_package(Boost REQUIRED COMPONENTS system)
find_package(TinyXML REQUIRED)

add_service_files(
  FILES
  write_8_outputs.srv
  request_command.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  trajectory_msgs
  sensor_msgs
)


catkin_package(
  INCLUDE_DIRS
    include
    ${catkin_INCLUDE_DIRS}
  LIBRARIES
    kuka_hardware_interface
    kuka_position_interface
    fdcc
  CATKIN_DEPENDS
    controller_manager
    hardware_interface
    joint_limits_interface
    roscpp
    std_msgs    
  DEPENDS
    TinyXML
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
  ${fdcc_INCLUDE_DIRS}
  ${fdcc_lib_INCLUDE_DIRS}
)

# Eigen
find_package(Eigen REQUIRED)
include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS} ${RBDL_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_library(kuka_hardware_interface
  src/kuka_hardware_interface.cpp
)

add_library(kuka_position_interface
  src/kuka_position_interface.cpp
)

target_link_libraries(kuka_hardware_interface
  fdcc_lib
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${TinyXML_LIBRARIES}
  ${EIGEN_LIBRARIES}
)

target_link_libraries(kuka_position_interface
  fdcc_lib
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${TinyXML_LIBRARIES}
  ${EIGEN_LIBRARIES}
)

add_executable(kuka_hardware_interface_node
  src/kuka_hardware_interface_node.cpp
)

target_link_libraries(kuka_hardware_interface_node 
  kuka_hardware_interface
)

add_executable(kuka_position_interface_node
  src/kuka_position_interface_node.cpp
)

target_link_libraries(kuka_position_interface_node 
  kuka_position_interface
)

install(
  TARGETS kuka_hardware_interface kuka_position_interface
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(
  TARGETS kuka_hardware_interface_node kuka_position_interface_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


