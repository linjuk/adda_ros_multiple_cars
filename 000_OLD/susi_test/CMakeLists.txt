cmake_minimum_required(VERSION 2.8.3)
project(susi_test)

set(CMAKE_CXX_STANDARD 11)
#set(CMAKE_BUILD_TYPE RelWithDebInfo)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy 
  tf 
  
)

catkin_package(
  LIBRARIES promp_robot_controller
  CATKIN_DEPENDS tf rospy 
)

