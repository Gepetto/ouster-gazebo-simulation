cmake_minimum_required(VERSION 2.8.3)
project(ouster_os1_64)

find_package(catkin REQUIRED)

catkin_package()

foreach(dir launch meshes urdf worlds)
  install(DIRECTORY ${dir} DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
endforeach()
