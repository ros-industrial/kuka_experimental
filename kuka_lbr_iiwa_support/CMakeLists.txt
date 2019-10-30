cmake_minimum_required(VERSION 2.8.3)

project(kuka_lbr_iiwa_support)

find_package(catkin REQUIRED)

catkin_package()

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(test/roslaunch_test_lbr_iiwa_7_r800.xml)
  roslaunch_add_file_check(test/roslaunch_test_lbr_iiwa_14_r820.xml)
endif()

install(DIRECTORY config launch meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
