cmake_minimum_required(VERSION 2.8.3)
project(kuka_eki_hw_interface)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  angles
  cmake_modules
  controller_manager
  hardware_interface
  joint_limits_interface
  roscpp
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(TinyXML REQUIRED)


catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    kuka_eki_hw_interface
  CATKIN_DEPENDS
    controller_manager
    hardware_interface
    joint_limits_interface
    roscpp
  DEPENDS
    TinyXML
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
)

add_library(kuka_eki_hw_interface
  src/kuka_eki_hw_interface.cpp
)

target_link_libraries(kuka_eki_hw_interface
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${TinyXML_LIBRARIES}
)

add_executable(kuka_eki_hw_interface_node
  src/kuka_eki_hw_interface_node.cpp
)

target_link_libraries(kuka_eki_hw_interface_node
  kuka_eki_hw_interface
)

install(
  TARGETS kuka_eki_hw_interface
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(
  TARGETS kuka_eki_hw_interface_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
