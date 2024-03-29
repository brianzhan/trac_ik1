cmake_minimum_required(VERSION 2.8.3)
project(trac_ik_kinematics_plugin)

if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
endif()

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  pluginlib
  roscpp
  tf_conversions
  trac_ik_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

catkin_package(
  LIBRARIES
  CATKIN_DEPENDS
    moveit_core
    pluginlib
    roscpp
    tf_conversions
    trac_ik_lib
)


set(TRAC_IK_LIBRARY_NAME trac_ik_kinematics_plugin)

add_library(${TRAC_IK_LIBRARY_NAME} src/trac_ik_kinematics_plugin.cpp)
target_link_libraries(${TRAC_IK_LIBRARY_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${TRAC_IK_LIBRARY_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(
  FILES
  trac_ik_kinematics_description.xml
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
)