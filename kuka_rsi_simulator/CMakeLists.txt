cmake_minimum_required(VERSION 3.0.2)
project(kuka_rsi_simulator)

find_package(catkin REQUIRED)

catkin_package()

install(
  PROGRAMS scripts/kuka_rsi_simulator
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
