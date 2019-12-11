# ROVI Project
This project is a part of the course Robotics and Computer Vision (ROVI) at University of Southern Denmark on the Advanced Robotics Technology master education.

This README explains the overall structure of the code.
For explanations of how to run the code, navigate into the folders and open the readme.txt located there.

To run the code in this project the following libraries needs to be installed:
- OpenCV 	Version 3.4
- PCL 		Version 1.4
- RobWork
- RobWorkStudio

OBS: if the error "buffer overflow detected" is present during execution of source files. Then the path to the
workcell used in the code might be too long. Try moving the workcell (and edit absolute paths) or move the whole project
to a location with a shorter path.

When compiling the cmake projects using RobWorkStudio a path to the RobWorkStudio root needs to be set in the cmakelists.txt
- set(RWSTUDIO_ROOT /home/emil/RobWork/RobWorkStudio)

## Folders
<b>Integration</b> contains workcell and code used for combination of robotics and computer vision solution.
- Sources is located at: ROVIProjekt/Integration/Project_WorkCell/SamplePluginPA10/src/
- SamplePlugin.cpp and SamplePlugin.hpp contains code for combining the robotics and vision solution.

<b>MATLAB</b> contains scripts which were used for data analysis of different tests performed.

<b>ObjectFiles</b> contains some of the object files used in this project for the workcells in RobWorkStudio.

<b>Project_WorkCell_Cam</b> contains a workcell with the duck object in the scene.

<b>Project_WorkCell_Cam_RRT</b> contains a workcell with the cylinder object and a bin around the pick area in the scene.

<b>ROB</b> contains folders with sources and test for reachability analysis and RRT-connect.
- Reachability source is located at ROVIProjekt/ROB/Reachability/src/
- RRT-connect source is located at ROVIProjekt/ROB/rrt_connect/src/


<b>VIS</b> contains folders with sources and test for M2 and M3.
- M2 sources is located at ROVIProjekt/VIS/M2/project/src/
- M3 sources is located at ROVIProjekt/VIS/SparseStereo/src/

<b>interpolator</b> contains folders with sources and test for linear interpolation with and without parabolic blend.
- Linear interpolation (with and without parabolic blend) sources is located at ROVIProjekt/interpolator/linear/src/



## Source files
The source files a short description and their location in the project folder is described in the table below.

|                       Name                       | Description                                                                                                        | Location                                                                       |
|:------------------------------------------------:|--------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------|
|               Reachability analysis              | Contains source code for implementation and  test of reachability                                                  | ROVIProjekt/ROB/Reachability/src/reachability.cpp                              |
|               Linear interpolation               | Contains functions which implements a linear interpolator                                                          | ROVIProjekt/interpolator/linear/src/interpolator.hpp                           |
|                  Parabolic blend                 | Contains functions implementing parabolic blend in a linear interpolator                                           | ROVIProjekt/interpolator/linear/src/interpolator.hpp                           |
| Test of linear interpolation and parabolic blend | File which should be run to execute tests of linear interpolation and parabolic blend                              | ROVIProjekt/interpolator/linear/src/linear.cpp                                 |
|                Test of RRT-connect               | File containing implmentation of tests and performance evaluation                                                  | ROVIProjekt/ROB/rrt_connect/src/main.cpp                                       |
|               RRT-connect utilities              | File containing general kinematic functions which is used in implementation                                        | ROVIProjekt/ROB/rrt_connect/src/util.hpp                                       |
|        RRT-connect performance evaluation        | File containing functions to evaluate performance primarily to determine optimal stepsize                          | ROVIProjekt/ROB/rrt_connect/src/performance_evaluation.hpp                     |
|                RRT-connect methods               | File containing two functions: one for planning between two configurations, one for planning between more than two | ROVIProjekt/ROB/rrt_connect/src/rrt_connect_methods.hpp                        |
|                 M2 preprocessing                 | Source code used for preprocessing  of scene in M2.                                                                | ROVIProjekt/VIS/M2/project/src/preprocess.hpp                                  |
|           M2 global alignment (RANSAC)           | Source code for implementation of global alignment                                                                 | ROVIProjekt/VIS/M2/project/src/global_alignment.hpp                            |
|             M2 local alignment (ICP)             | Source code for implementation of local alignment                                                                  | ROVIProjekt/VIS/M2/project/src/local_alignment.hpp                             |
|                   M2 alignment                   | File including global and local alignment in correct namespace                                                     | ROVIProjekt/VIS/M2/project/src/alignment.hpp                                   |
|                M2 pose estimation                | Source code for pose estimation. Calls functions from other M2 source files to perform pose estimation.            | ROVIProjekt/VIS/M2/project/src/pose_estimation.hpp                             |
|                    Test of M2                    | Implementation of tests for M2 including example of pose estimate from two point clouds.                           | ROVIProjekt/VIS/M2/project/src/main.cpp                                        |
|              3D and 6D version of M3             | Contains implementation of methods for M3                                                                          | ROVIProjekt/VIS/SparseStereo/src/SparseStereoMethods.hpp                       |
|             Functions for test of M3             | Contains functions to test 3D and 6D version of M3                                                                 | ROVIProjekt/VIS/SparseStereo/src/TestingMethods.hpp                            |
|                    Test of M3                    | Implementation of tests for both versions of M3.                                                                   | ROVIProjekt/VIS/SparseStereo/src/main.cpp                                      |
|                Integration plugin                | Implementation of combined vision and robotics solution and plugin for visualization.                              | ROVIProjekt/Integration/Project_WorkCell/SamplePluginPA10/src/SamplePlugin.hpp |
|                Integration plugin                | Implementation of combined vision and robotics solution and plugin for visualization.                              | ROVIProjekt/Integration/Project_WorkCell/SamplePluginPA10/src/SamplePlugin.cpp |
