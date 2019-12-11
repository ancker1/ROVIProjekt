cmake_minimum_required(VERSION 3.1)
set (CMAKE_CXX_STANDARD 11)
# Project name
project( rrt_connect )

# Used for: Resolve absolute path names
SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

SET(RW_ROOT ~/RobWork/RobWork)


SET(CMAKE_BUILD_TYPE Debug)
SET(RobWork_DIR ${RW_ROOT}/cmake)

FIND_PACKAGE(RobWork REQUIRED)

include_directories(${ROBWORK_INCLUDE_DIRS})
link_directories(${ROBWORK_LIBRARY_DIRS})

add_executable( rrt_connect src/main.cpp src/rrt_connect_methods.hpp src/util.hpp src/performance_evaluation.hpp)
target_link_libraries(rrt_connect ${ROBWORK_LIBRARIES})
 
