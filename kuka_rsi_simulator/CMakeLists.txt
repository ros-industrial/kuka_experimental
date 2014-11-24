cmake_minimum_required(VERSION 2.8.3)
project(kuka_rsi_simulator)

find_package(catkin REQUIRED)

catkin_package()

install(PROGRAMS kuka_rsi_simulator DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
