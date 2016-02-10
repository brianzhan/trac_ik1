cmake_minimum_required(VERSION 2.8.3)
project(trac_ik_trajectories)

if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
endif()


find_package(catkin REQUIRED COMPONENTS
  roscpp
  trac_ik_lib
  eigen_conversions
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp trac_ik_lib eigen_conversions 
  LIBRARIES trac_ik_trajectories
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(TRAC_IK_LIBRARY_NAME trac_ik_trajectories)

add_library(${TRAC_IK_LIBRARY_NAME} src/cartesian_trajectories.cpp)

target_link_libraries(${TRAC_IK_LIBRARY_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${TRAC_IK_LIBRARY_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)