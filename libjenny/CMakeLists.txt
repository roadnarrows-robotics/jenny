cmake_minimum_required(VERSION 2.8.3)
project(libjenny)

find_package(catkin 
  REQUIRED COMPONENTS 
  actionlib_msgs
  std_msgs 
  genmsg 
  message_generation 
  roscpp 
  rospy 
)

find_package(Boost REQUIRED COMPONENTS system)

set(rnr-jenny_DIR /usr/local/lib/cmake/rnr)
find_package(rnr-jenny
  REQUIRED
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs
)

include_directories(
  include 
  ../include
  ${rnr-jenny_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

set(RN_LOGGING_COMPILE_FLAGS "-DLOG -DLOGMOD=\\\"libjenny\\\"")
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${RN_LOGGING_COMPILE_FLAGS}")
#message(DBG: ${CMAKE_CXX_FLAGS})

add_library(jenny
  src/naze32imu.cxx
  src/robot.cxx
  src/rs160d.cxx
  src/utils.cxx
)

target_link_libraries(jenny 
  ${rnr-jenny_LIBRARIES}
  ${catkin_LIBRARIES}
)

install(TARGETS jenny
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY ../include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)
