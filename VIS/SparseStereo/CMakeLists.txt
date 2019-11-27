cmake_minimum_required(VERSION 3.1)
set (CMAKE_CXX_STANDARD 11)
# Project name
project( SparseStereo )

# Used for: Resolve absolute path names
SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

SET(RW_ROOT ~/RobWork/RobWork)
set(RWSTUDIO_ROOT ~/RobWork/RobWorkStudio)

SET(CMAKE_BUILD_TYPE Debug)
SET(RobWork_DIR ${RW_ROOT}/cmake)
set(RobWorkStudio_DIR ${RWSTUDIO_ROOT}/cmake)

FIND_PACKAGE(RobWork REQUIRED)
find_package(RobWorkStudio REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${ROBWORK_INCLUDE_DIRS} ${ROBWORKSTUDIO_INCLUDE_DIRS})
link_directories(${ROBWORK_LIBRARY_DIRS} ${ROBWORKSTUDIO_LIBRARY_DIRS})

add_executable( SparseStereo src/SparseStereo.cpp )
target_link_libraries(SparseStereo ${OpenCV_LIBS} ${ROBWORK_LIBRARIES} ${ROBWORKSTUDIO_LIBRARIES})
 
