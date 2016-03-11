cmake_minimum_required(VERSION 2.8.3)
project(kuka_lbr_iiwa_support)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY config launch urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# temporarily: until sources are moved to other org/rep
install(
  DIRECTORY
    meshes/lbr_iiwa_14_r820/collision
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes/lbr_iiwa_14_r820/)

# temporarily: until sources are moved to other org/rep
install(
  DIRECTORY
    meshes/lbr_iiwa_14_r820/visual
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes/lbr_iiwa_14_r820/)
