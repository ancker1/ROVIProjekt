# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/mikkel/Desktop/Project_WorkCell_Cam/RoviSamplePlugin

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/mikkel/Desktop/Project_WorkCell_Cam/RoviSamplePlugin/build

# Include any dependencies generated for this target.
include CMakeFiles/RoViPlugin.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/RoViPlugin.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/RoViPlugin.dir/flags.make

ui_SamplePlugin.h: ../src/SamplePlugin.ui
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/mikkel/Desktop/Project_WorkCell_Cam/RoviSamplePlugin/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating ui_SamplePlugin.h"
	/usr/lib/qt5/bin/uic -o /home/mikkel/Desktop/Project_WorkCell_Cam/RoviSamplePlugin/build/ui_SamplePlugin.h /home/mikkel/Desktop/Project_WorkCell_Cam/RoviSamplePlugin/src/SamplePlugin.ui

src/moc_SamplePlugin.cpp: ../src/SamplePlugin.hpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/mikkel/Desktop/Project_WorkCell_Cam/RoviSamplePlugin/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Generating src/moc_SamplePlugin.cpp"
	cd /home/mikkel/Desktop/Project_WorkCell_Cam/RoviSamplePlugin/build/src && /usr/lib/qt5/bin/moc @/home/mikkel/Desktop/Project_WorkCell_Cam/RoviSamplePlugin/build/src/moc_SamplePlugin.cpp_parameters

qrc_resources.cpp: ../src/pa_icon.png
qrc_resources.cpp: ../src/resources.qrc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/mikkel/Desktop/Project_WorkCell_Cam/RoviSamplePlugin/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Generating qrc_resources.cpp"
	/usr/lib/qt5/bin/rcc --name resources --output /home/mikkel/Desktop/Project_WorkCell_Cam/RoviSamplePlugin/build/qrc_resources.cpp /home/mikkel/Desktop/Project_WorkCell_Cam/RoviSamplePlugin/src/resources.qrc

CMakeFiles/RoViPlugin.dir/src/SamplePlugin.cpp.o: CMakeFiles/RoViPlugin.dir/flags.make
CMakeFiles/RoViPlugin.dir/src/SamplePlugin.cpp.o: ../src/SamplePlugin.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/mikkel/Desktop/Project_WorkCell_Cam/RoviSamplePlugin/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/RoViPlugin.dir/src/SamplePlugin.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/RoViPlugin.dir/src/SamplePlugin.cpp.o -c /home/mikkel/Desktop/Project_WorkCell_Cam/RoviSamplePlugin/src/SamplePlugin.cpp

CMakeFiles/RoViPlugin.dir/src/SamplePlugin.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/RoViPlugin.dir/src/SamplePlugin.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/mikkel/Desktop/Project_WorkCell_Cam/RoviSamplePlugin/src/SamplePlugin.cpp > CMakeFiles/RoViPlugin.dir/src/SamplePlugin.cpp.i

CMakeFiles/RoViPlugin.dir/src/SamplePlugin.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/RoViPlugin.dir/src/SamplePlugin.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/mikkel/Desktop/Project_WorkCell_Cam/RoviSamplePlugin/src/SamplePlugin.cpp -o CMakeFiles/RoViPlugin.dir/src/SamplePlugin.cpp.s

CMakeFiles/RoViPlugin.dir/src/SamplePlugin.cpp.o.requires:

.PHONY : CMakeFiles/RoViPlugin.dir/src/SamplePlugin.cpp.o.requires

CMakeFiles/RoViPlugin.dir/src/SamplePlugin.cpp.o.provides: CMakeFiles/RoViPlugin.dir/src/SamplePlugin.cpp.o.requires
	$(MAKE) -f CMakeFiles/RoViPlugin.dir/build.make CMakeFiles/RoViPlugin.dir/src/SamplePlugin.cpp.o.provides.build
.PHONY : CMakeFiles/RoViPlugin.dir/src/SamplePlugin.cpp.o.provides

CMakeFiles/RoViPlugin.dir/src/SamplePlugin.cpp.o.provides.build: CMakeFiles/RoViPlugin.dir/src/SamplePlugin.cpp.o


CMakeFiles/RoViPlugin.dir/src/moc_SamplePlugin.cpp.o: CMakeFiles/RoViPlugin.dir/flags.make
CMakeFiles/RoViPlugin.dir/src/moc_SamplePlugin.cpp.o: src/moc_SamplePlugin.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/mikkel/Desktop/Project_WorkCell_Cam/RoviSamplePlugin/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/RoViPlugin.dir/src/moc_SamplePlugin.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/RoViPlugin.dir/src/moc_SamplePlugin.cpp.o -c /home/mikkel/Desktop/Project_WorkCell_Cam/RoviSamplePlugin/build/src/moc_SamplePlugin.cpp

CMakeFiles/RoViPlugin.dir/src/moc_SamplePlugin.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/RoViPlugin.dir/src/moc_SamplePlugin.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/mikkel/Desktop/Project_WorkCell_Cam/RoviSamplePlugin/build/src/moc_SamplePlugin.cpp > CMakeFiles/RoViPlugin.dir/src/moc_SamplePlugin.cpp.i

CMakeFiles/RoViPlugin.dir/src/moc_SamplePlugin.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/RoViPlugin.dir/src/moc_SamplePlugin.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/mikkel/Desktop/Project_WorkCell_Cam/RoviSamplePlugin/build/src/moc_SamplePlugin.cpp -o CMakeFiles/RoViPlugin.dir/src/moc_SamplePlugin.cpp.s

CMakeFiles/RoViPlugin.dir/src/moc_SamplePlugin.cpp.o.requires:

.PHONY : CMakeFiles/RoViPlugin.dir/src/moc_SamplePlugin.cpp.o.requires

CMakeFiles/RoViPlugin.dir/src/moc_SamplePlugin.cpp.o.provides: CMakeFiles/RoViPlugin.dir/src/moc_SamplePlugin.cpp.o.requires
	$(MAKE) -f CMakeFiles/RoViPlugin.dir/build.make CMakeFiles/RoViPlugin.dir/src/moc_SamplePlugin.cpp.o.provides.build
.PHONY : CMakeFiles/RoViPlugin.dir/src/moc_SamplePlugin.cpp.o.provides

CMakeFiles/RoViPlugin.dir/src/moc_SamplePlugin.cpp.o.provides.build: CMakeFiles/RoViPlugin.dir/src/moc_SamplePlugin.cpp.o


CMakeFiles/RoViPlugin.dir/qrc_resources.cpp.o: CMakeFiles/RoViPlugin.dir/flags.make
CMakeFiles/RoViPlugin.dir/qrc_resources.cpp.o: qrc_resources.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/mikkel/Desktop/Project_WorkCell_Cam/RoviSamplePlugin/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/RoViPlugin.dir/qrc_resources.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/RoViPlugin.dir/qrc_resources.cpp.o -c /home/mikkel/Desktop/Project_WorkCell_Cam/RoviSamplePlugin/build/qrc_resources.cpp

CMakeFiles/RoViPlugin.dir/qrc_resources.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/RoViPlugin.dir/qrc_resources.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/mikkel/Desktop/Project_WorkCell_Cam/RoviSamplePlugin/build/qrc_resources.cpp > CMakeFiles/RoViPlugin.dir/qrc_resources.cpp.i

CMakeFiles/RoViPlugin.dir/qrc_resources.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/RoViPlugin.dir/qrc_resources.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/mikkel/Desktop/Project_WorkCell_Cam/RoviSamplePlugin/build/qrc_resources.cpp -o CMakeFiles/RoViPlugin.dir/qrc_resources.cpp.s

CMakeFiles/RoViPlugin.dir/qrc_resources.cpp.o.requires:

.PHONY : CMakeFiles/RoViPlugin.dir/qrc_resources.cpp.o.requires

CMakeFiles/RoViPlugin.dir/qrc_resources.cpp.o.provides: CMakeFiles/RoViPlugin.dir/qrc_resources.cpp.o.requires
	$(MAKE) -f CMakeFiles/RoViPlugin.dir/build.make CMakeFiles/RoViPlugin.dir/qrc_resources.cpp.o.provides.build
.PHONY : CMakeFiles/RoViPlugin.dir/qrc_resources.cpp.o.provides

CMakeFiles/RoViPlugin.dir/qrc_resources.cpp.o.provides.build: CMakeFiles/RoViPlugin.dir/qrc_resources.cpp.o


# Object files for target RoViPlugin
RoViPlugin_OBJECTS = \
"CMakeFiles/RoViPlugin.dir/src/SamplePlugin.cpp.o" \
"CMakeFiles/RoViPlugin.dir/src/moc_SamplePlugin.cpp.o" \
"CMakeFiles/RoViPlugin.dir/qrc_resources.cpp.o"

# External object files for target RoViPlugin
RoViPlugin_EXTERNAL_OBJECTS =

../libs/Release/libRoViPlugin.so: CMakeFiles/RoViPlugin.dir/src/SamplePlugin.cpp.o
../libs/Release/libRoViPlugin.so: CMakeFiles/RoViPlugin.dir/src/moc_SamplePlugin.cpp.o
../libs/Release/libRoViPlugin.so: CMakeFiles/RoViPlugin.dir/qrc_resources.cpp.o
../libs/Release/libRoViPlugin.so: CMakeFiles/RoViPlugin.dir/build.make
../libs/Release/libRoViPlugin.so: /usr/lib/x86_64-linux-gnu/libpython3.6m.so
../libs/Release/libRoViPlugin.so: /home/mikkel/RobWork/RobWork/libs/release/libsdurw_lua_s.a
../libs/Release/libRoViPlugin.so: /home/mikkel/RobWork/RobWork/libs/release/libsdurw_assembly_lua_s.a
../libs/Release/libRoViPlugin.so: /home/mikkel/RobWork/RobWork/libs/release/libsdurw_control_lua_s.a
../libs/Release/libRoViPlugin.so: /home/mikkel/RobWork/RobWork/libs/release/libsdurw_pathoptimization_lua_s.a
../libs/Release/libRoViPlugin.so: /home/mikkel/RobWork/RobWork/libs/release/libsdurw_pathplanners_lua_s.a
../libs/Release/libRoViPlugin.so: /home/mikkel/RobWork/RobWork/libs/release/libsdurw_proximitystrategies_lua_s.a
../libs/Release/libRoViPlugin.so: /home/mikkel/RobWork/RobWork/libs/release/libsdurw_simulation_lua_s.a
../libs/Release/libRoViPlugin.so: /home/mikkel/RobWork/RobWork/libs/release/libsdurw_task_lua_s.a
../libs/Release/libRoViPlugin.so: /usr/lib/x86_64-linux-gnu/liblua5.3.so
../libs/Release/libRoViPlugin.so: /usr/lib/x86_64-linux-gnu/libm.so
../libs/Release/libRoViPlugin.so: /home/mikkel/RobWork/RobWork/libs/release/libsdurw_algorithms.so
../libs/Release/libRoViPlugin.so: /home/mikkel/RobWork/RobWork/libs/release/libsdurw_pathplanners.so
../libs/Release/libRoViPlugin.so: /home/mikkel/RobWork/RobWork/libs/release/libsdurw_pathoptimization.so
../libs/Release/libRoViPlugin.so: /home/mikkel/RobWork/RobWork/libs/release/libsdurw_simulation.so
../libs/Release/libRoViPlugin.so: /home/mikkel/RobWork/RobWork/libs/release/libsdurw_opengl.so
../libs/Release/libRoViPlugin.so: /home/mikkel/RobWork/RobWork/libs/release/libsdurw_assembly.so
../libs/Release/libRoViPlugin.so: /home/mikkel/RobWork/RobWork/libs/release/libsdurw_task.so
../libs/Release/libRoViPlugin.so: /home/mikkel/RobWork/RobWork/libs/release/libsdurw_calibration.so
../libs/Release/libRoViPlugin.so: /home/mikkel/RobWork/RobWork/libs/release/libsdurw_csg.so
../libs/Release/libRoViPlugin.so: /home/mikkel/RobWork/RobWork/libs/release/libsdurw_control.so
../libs/Release/libRoViPlugin.so: /home/mikkel/RobWork/RobWork/libs/release/libsdurw_proximitystrategies.so
../libs/Release/libRoViPlugin.so: /home/mikkel/RobWork/RobWork/libs/release/libyaobi.so
../libs/Release/libRoViPlugin.so: /home/mikkel/RobWork/RobWork/libs/release/libpqp.so
../libs/Release/libRoViPlugin.so: /home/mikkel/RobWork/RobWork/libs/release/libfcl.so
../libs/Release/libRoViPlugin.so: /home/mikkel/RobWork/RobWork/libs/release/libsdurw.so
../libs/Release/libRoViPlugin.so: /usr/lib/x86_64-linux-gnu/libGL.so
../libs/Release/libRoViPlugin.so: /usr/lib/x86_64-linux-gnu/libGLU.so
../libs/Release/libRoViPlugin.so: /usr/lib/x86_64-linux-gnu/libxerces-c.so
../libs/Release/libRoViPlugin.so: /home/mikkel/RobWork/RobWork/libs/release/libsdurw_assimp.so
../libs/Release/libRoViPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
../libs/Release/libRoViPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so
../libs/Release/libRoViPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
../libs/Release/libRoViPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
../libs/Release/libRoViPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
../libs/Release/libRoViPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
../libs/Release/libRoViPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
../libs/Release/libRoViPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
../libs/Release/libRoViPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
../libs/Release/libRoViPlugin.so: /usr/lib/x86_64-linux-gnu/libpthread.so
../libs/Release/libRoViPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_test_exec_monitor.a
../libs/Release/libRoViPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_unit_test_framework.so
../libs/Release/libRoViPlugin.so: /home/mikkel/RobWork/RobWork/libs/release/libsdurw_unzip.a
../libs/Release/libRoViPlugin.so: /usr/lib/x86_64-linux-gnu/libz.so
../libs/Release/libRoViPlugin.so: /usr/lib/x86_64-linux-gnu/libdl.so
../libs/Release/libRoViPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
../libs/Release/libRoViPlugin.so: /usr/lib/x86_64-linux-gnu/libGL.so
../libs/Release/libRoViPlugin.so: /usr/lib/x86_64-linux-gnu/libGLU.so
../libs/Release/libRoViPlugin.so: /home/mikkel/RobWork/RobWork/libs/release/libsdurw_lua_s.a
../libs/Release/libRoViPlugin.so: /home/mikkel/RobWork/RobWork/libs/release/libsdurw_assembly_lua_s.a
../libs/Release/libRoViPlugin.so: /home/mikkel/RobWork/RobWork/libs/release/libsdurw_control_lua_s.a
../libs/Release/libRoViPlugin.so: /home/mikkel/RobWork/RobWork/libs/release/libsdurw_pathoptimization_lua_s.a
../libs/Release/libRoViPlugin.so: /home/mikkel/RobWork/RobWork/libs/release/libsdurw_pathplanners_lua_s.a
../libs/Release/libRoViPlugin.so: /home/mikkel/RobWork/RobWork/libs/release/libsdurw_proximitystrategies_lua_s.a
../libs/Release/libRoViPlugin.so: /home/mikkel/RobWork/RobWork/libs/release/libsdurw_simulation_lua_s.a
../libs/Release/libRoViPlugin.so: /home/mikkel/RobWork/RobWork/libs/release/libsdurw_task_lua_s.a
../libs/Release/libRoViPlugin.so: /usr/lib/x86_64-linux-gnu/liblua5.3.so
../libs/Release/libRoViPlugin.so: /usr/lib/x86_64-linux-gnu/libm.so
../libs/Release/libRoViPlugin.so: /home/mikkel/RobWork/RobWork/libs/release/libsdurw_algorithms.so
../libs/Release/libRoViPlugin.so: /home/mikkel/RobWork/RobWork/libs/release/libsdurw_pathplanners.so
../libs/Release/libRoViPlugin.so: /home/mikkel/RobWork/RobWork/libs/release/libsdurw_pathoptimization.so
../libs/Release/libRoViPlugin.so: /home/mikkel/RobWork/RobWork/libs/release/libsdurw_simulation.so
../libs/Release/libRoViPlugin.so: /home/mikkel/RobWork/RobWork/libs/release/libsdurw_opengl.so
../libs/Release/libRoViPlugin.so: /home/mikkel/RobWork/RobWork/libs/release/libsdurw_assembly.so
../libs/Release/libRoViPlugin.so: /home/mikkel/RobWork/RobWork/libs/release/libsdurw_task.so
../libs/Release/libRoViPlugin.so: /home/mikkel/RobWork/RobWork/libs/release/libsdurw_calibration.so
../libs/Release/libRoViPlugin.so: /home/mikkel/RobWork/RobWork/libs/release/libsdurw_csg.so
../libs/Release/libRoViPlugin.so: /home/mikkel/RobWork/RobWork/libs/release/libsdurw_control.so
../libs/Release/libRoViPlugin.so: /home/mikkel/RobWork/RobWork/libs/release/libsdurw_proximitystrategies.so
../libs/Release/libRoViPlugin.so: /home/mikkel/RobWork/RobWork/libs/release/libyaobi.so
../libs/Release/libRoViPlugin.so: /home/mikkel/RobWork/RobWork/libs/release/libpqp.so
../libs/Release/libRoViPlugin.so: /home/mikkel/RobWork/RobWork/libs/release/libfcl.so
../libs/Release/libRoViPlugin.so: /home/mikkel/RobWork/RobWork/libs/release/libsdurw.so
../libs/Release/libRoViPlugin.so: /usr/lib/x86_64-linux-gnu/libGL.so
../libs/Release/libRoViPlugin.so: /usr/lib/x86_64-linux-gnu/libGLU.so
../libs/Release/libRoViPlugin.so: /usr/lib/x86_64-linux-gnu/libxerces-c.so
../libs/Release/libRoViPlugin.so: /home/mikkel/RobWork/RobWork/libs/release/libsdurw_assimp.so
../libs/Release/libRoViPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
../libs/Release/libRoViPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so
../libs/Release/libRoViPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
../libs/Release/libRoViPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
../libs/Release/libRoViPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
../libs/Release/libRoViPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
../libs/Release/libRoViPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
../libs/Release/libRoViPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
../libs/Release/libRoViPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
../libs/Release/libRoViPlugin.so: /usr/lib/x86_64-linux-gnu/libpthread.so
../libs/Release/libRoViPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_test_exec_monitor.a
../libs/Release/libRoViPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_unit_test_framework.so
../libs/Release/libRoViPlugin.so: /home/mikkel/RobWork/RobWork/libs/release/libsdurw_unzip.a
../libs/Release/libRoViPlugin.so: /usr/lib/x86_64-linux-gnu/libz.so
../libs/Release/libRoViPlugin.so: /usr/lib/x86_64-linux-gnu/libdl.so
../libs/Release/libRoViPlugin.so: /usr/local/lib/libopencv_stitching.so.3.4.7
../libs/Release/libRoViPlugin.so: /usr/local/lib/libopencv_superres.so.3.4.7
../libs/Release/libRoViPlugin.so: /usr/local/lib/libopencv_videostab.so.3.4.7
../libs/Release/libRoViPlugin.so: /usr/local/lib/libopencv_aruco.so.3.4.7
../libs/Release/libRoViPlugin.so: /usr/local/lib/libopencv_bgsegm.so.3.4.7
../libs/Release/libRoViPlugin.so: /usr/local/lib/libopencv_bioinspired.so.3.4.7
../libs/Release/libRoViPlugin.so: /usr/local/lib/libopencv_ccalib.so.3.4.7
../libs/Release/libRoViPlugin.so: /usr/local/lib/libopencv_cvv.so.3.4.7
../libs/Release/libRoViPlugin.so: /usr/local/lib/libopencv_dnn_objdetect.so.3.4.7
../libs/Release/libRoViPlugin.so: /usr/local/lib/libopencv_dpm.so.3.4.7
../libs/Release/libRoViPlugin.so: /usr/local/lib/libopencv_face.so.3.4.7
../libs/Release/libRoViPlugin.so: /usr/local/lib/libopencv_freetype.so.3.4.7
../libs/Release/libRoViPlugin.so: /usr/local/lib/libopencv_fuzzy.so.3.4.7
../libs/Release/libRoViPlugin.so: /usr/local/lib/libopencv_hdf.so.3.4.7
../libs/Release/libRoViPlugin.so: /usr/local/lib/libopencv_hfs.so.3.4.7
../libs/Release/libRoViPlugin.so: /usr/local/lib/libopencv_img_hash.so.3.4.7
../libs/Release/libRoViPlugin.so: /usr/local/lib/libopencv_line_descriptor.so.3.4.7
../libs/Release/libRoViPlugin.so: /usr/local/lib/libopencv_optflow.so.3.4.7
../libs/Release/libRoViPlugin.so: /usr/local/lib/libopencv_reg.so.3.4.7
../libs/Release/libRoViPlugin.so: /usr/local/lib/libopencv_rgbd.so.3.4.7
../libs/Release/libRoViPlugin.so: /usr/local/lib/libopencv_saliency.so.3.4.7
../libs/Release/libRoViPlugin.so: /usr/local/lib/libopencv_sfm.so.3.4.7
../libs/Release/libRoViPlugin.so: /usr/local/lib/libopencv_stereo.so.3.4.7
../libs/Release/libRoViPlugin.so: /usr/local/lib/libopencv_structured_light.so.3.4.7
../libs/Release/libRoViPlugin.so: /usr/local/lib/libopencv_surface_matching.so.3.4.7
../libs/Release/libRoViPlugin.so: /usr/local/lib/libopencv_tracking.so.3.4.7
../libs/Release/libRoViPlugin.so: /usr/local/lib/libopencv_xfeatures2d.so.3.4.7
../libs/Release/libRoViPlugin.so: /usr/local/lib/libopencv_ximgproc.so.3.4.7
../libs/Release/libRoViPlugin.so: /usr/local/lib/libopencv_xobjdetect.so.3.4.7
../libs/Release/libRoViPlugin.so: /usr/local/lib/libopencv_xphoto.so.3.4.7
../libs/Release/libRoViPlugin.so: /usr/local/lib/libopencv_shape.so.3.4.7
../libs/Release/libRoViPlugin.so: /usr/local/lib/libopencv_viz.so.3.4.7
../libs/Release/libRoViPlugin.so: /usr/local/lib/libopencv_phase_unwrapping.so.3.4.7
../libs/Release/libRoViPlugin.so: /usr/local/lib/libopencv_video.so.3.4.7
../libs/Release/libRoViPlugin.so: /usr/local/lib/libopencv_datasets.so.3.4.7
../libs/Release/libRoViPlugin.so: /usr/local/lib/libopencv_plot.so.3.4.7
../libs/Release/libRoViPlugin.so: /usr/local/lib/libopencv_text.so.3.4.7
../libs/Release/libRoViPlugin.so: /usr/local/lib/libopencv_dnn.so.3.4.7
../libs/Release/libRoViPlugin.so: /usr/local/lib/libopencv_highgui.so.3.4.7
../libs/Release/libRoViPlugin.so: /usr/lib/x86_64-linux-gnu/libQt5OpenGL.so.5.9.5
../libs/Release/libRoViPlugin.so: /usr/lib/x86_64-linux-gnu/libQt5Widgets.so.5.9.5
../libs/Release/libRoViPlugin.so: /usr/lib/x86_64-linux-gnu/libQt5Gui.so.5.9.5
../libs/Release/libRoViPlugin.so: /usr/lib/x86_64-linux-gnu/libQt5Core.so.5.9.5
../libs/Release/libRoViPlugin.so: /usr/local/lib/libopencv_ml.so.3.4.7
../libs/Release/libRoViPlugin.so: /usr/local/lib/libopencv_videoio.so.3.4.7
../libs/Release/libRoViPlugin.so: /usr/local/lib/libopencv_imgcodecs.so.3.4.7
../libs/Release/libRoViPlugin.so: /usr/local/lib/libopencv_objdetect.so.3.4.7
../libs/Release/libRoViPlugin.so: /usr/local/lib/libopencv_calib3d.so.3.4.7
../libs/Release/libRoViPlugin.so: /usr/local/lib/libopencv_features2d.so.3.4.7
../libs/Release/libRoViPlugin.so: /usr/local/lib/libopencv_flann.so.3.4.7
../libs/Release/libRoViPlugin.so: /usr/local/lib/libopencv_photo.so.3.4.7
../libs/Release/libRoViPlugin.so: /usr/local/lib/libopencv_imgproc.so.3.4.7
../libs/Release/libRoViPlugin.so: /usr/local/lib/libopencv_core.so.3.4.7
../libs/Release/libRoViPlugin.so: CMakeFiles/RoViPlugin.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/mikkel/Desktop/Project_WorkCell_Cam/RoviSamplePlugin/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Linking CXX shared module ../libs/Release/libRoViPlugin.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/RoViPlugin.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/RoViPlugin.dir/build: ../libs/Release/libRoViPlugin.so

.PHONY : CMakeFiles/RoViPlugin.dir/build

CMakeFiles/RoViPlugin.dir/requires: CMakeFiles/RoViPlugin.dir/src/SamplePlugin.cpp.o.requires
CMakeFiles/RoViPlugin.dir/requires: CMakeFiles/RoViPlugin.dir/src/moc_SamplePlugin.cpp.o.requires
CMakeFiles/RoViPlugin.dir/requires: CMakeFiles/RoViPlugin.dir/qrc_resources.cpp.o.requires

.PHONY : CMakeFiles/RoViPlugin.dir/requires

CMakeFiles/RoViPlugin.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/RoViPlugin.dir/cmake_clean.cmake
.PHONY : CMakeFiles/RoViPlugin.dir/clean

CMakeFiles/RoViPlugin.dir/depend: ui_SamplePlugin.h
CMakeFiles/RoViPlugin.dir/depend: src/moc_SamplePlugin.cpp
CMakeFiles/RoViPlugin.dir/depend: qrc_resources.cpp
	cd /home/mikkel/Desktop/Project_WorkCell_Cam/RoviSamplePlugin/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/mikkel/Desktop/Project_WorkCell_Cam/RoviSamplePlugin /home/mikkel/Desktop/Project_WorkCell_Cam/RoviSamplePlugin /home/mikkel/Desktop/Project_WorkCell_Cam/RoviSamplePlugin/build /home/mikkel/Desktop/Project_WorkCell_Cam/RoviSamplePlugin/build /home/mikkel/Desktop/Project_WorkCell_Cam/RoviSamplePlugin/build/CMakeFiles/RoViPlugin.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/RoViPlugin.dir/depend

