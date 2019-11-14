cmake_minimum_required(VERSION 3.1)
set (CMAKE_CXX_STANDARD 11)
# Project name
project( reachability )

# Used for: Resolve absolute path names
SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

SET(RW_ROOT ~/RobWork/RobWork)


SET(CMAKE_BUILD_TYPE Debug)
SET(RobWork_DIR ${RW_ROOT}/cmake)

FIND_PACKAGE(RobWork REQUIRED)

include_directories(${ROBWORK_INCLUDE_DIRS})
link_directories(${ROBWORK_LIBRARY_DIRS})

add_executable( reachability src/reachability.cpp)
target_link_libraries(reachability ${ROBWORK_LIBRARIES})
 
