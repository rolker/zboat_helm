cmake_minimum_required(VERSION 2.8.3)
project(zboat_helm)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  geographic_msgs
  geometry_msgs
  rospy
  dynamic_reconfigure
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
    cfg/zboat_helm.cfg
)


catkin_package(
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS nodes/zboat_helm_node.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
