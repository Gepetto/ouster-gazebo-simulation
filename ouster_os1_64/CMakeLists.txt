cmake_minimum_required(VERSION 3.1)
project(ouster_os1_64)

find_package(catkin REQUIRED)

catkin_package()

foreach(dir launch meshes urdf worlds)
  install(DIRECTORY ${dir} DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
endforeach()
