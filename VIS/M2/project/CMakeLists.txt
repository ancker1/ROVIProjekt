cmake_minimum_required(VERSION 3.1)
set (CMAKE_CXX_STANDARD 11)
# Project name
project( M2 )

# Used for: Resolve absolute path names
SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

SET(RW_ROOT /home/emil/RobWork/RobWork)
set(RWSTUDIO_ROOT /home/emil/RobWork/RobWorkStudio)

SET(CMAKE_BUILD_TYPE Release)
SET(RobWork_DIR ${RW_ROOT}/cmake)
set(RobWorkStudio_DIR ${RWSTUDIO_ROOT}/cmake)


FIND_PACKAGE(RobWork REQUIRED)
find_package(RobWorkStudio REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL 1.4 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${ROBWORK_INCLUDE_DIRS} ${ROBWORKSTUDIO_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${ROBWORK_LIBRARY_DIRS} ${ROBWORKSTUDIO_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable( M2 src/main.cpp src/global_alignment.hpp src/local_alignment.hpp src/alignment.hpp src/preprocess.hpp)
target_link_libraries(M2 ${PCL_LIBRARIES} ${OpenCV_LIBS} ${ROBWORK_LIBRARIES} ${ROBWORKSTUDIO_LIBRARIES})
 
