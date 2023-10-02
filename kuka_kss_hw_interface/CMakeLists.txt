cmake_minimum_required(VERSION 3.0.2)
project(kuka_kss_hw_interface)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g")

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  controller_manager
  hardware_interface
  joint_limits_interface
  realtime_tools
  roscpp
  std_msgs
  industrial_msgs
  kuka_msgs
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(TinyXML REQUIRED)


catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    kuka_kss_hw_interface
  CATKIN_DEPENDS
    controller_manager
    hardware_interface
    joint_limits_interface
    roscpp
    std_msgs
    industrial_msgs
    kuka_msgs
  DEPENDS
    TinyXML
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
)

add_library(kuka_kss_hw_interface
  src/XmlTemplate.cpp
  src/XmlTemplateXPath.cpp
  src/communication_link.cpp
  src/comm_link_udp_server.cpp
  src/comm_link_tcp_client.cpp
  src/kuka_comm_handler.cpp
  src/kuka_comm_handler_eki.cpp
  src/kuka_comm_handler_rsi.cpp
  src/kuka_robot_state.cpp
  src/kuka_robot_state_manager.cpp
  src/kuka_kss_hw_interface.cpp
)

target_link_libraries(kuka_kss_hw_interface
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${TinyXML_LIBRARIES}
)

add_executable(kuka_kss_hw_interface_node
  src/kuka_kss_hw_interface_node.cpp
)

target_link_libraries(kuka_kss_hw_interface_node
   kuka_kss_hw_interface
)

install(
  TARGETS kuka_kss_hw_interface
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(
  TARGETS kuka_kss_hw_interface_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
