cmake_minimum_required(VERSION 3.1)
set (CMAKE_CXX_STANDARD 11)
# Project name
project( RRT )

# Used for: Resolve absolute path names
SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

SET(RW_ROOT /home/emil/RobWork/RobWork)


SET(CMAKE_BUILD_TYPE Release)
SET(RobWork_DIR ${RW_ROOT}/cmake)

FIND_PACKAGE(RobWork REQUIRED)

include_directories(${ROBWORK_INCLUDE_DIRS})
link_directories(${ROBWORK_LIBRARY_DIRS})

add_executable( RRT src/RRT.cpp)
target_link_libraries(RRT ${ROBWORK_LIBRARIES})
 
