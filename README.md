# ROVIProjekt
This README explains the overall structure of the code.
For explanations of how to run the code, navigate into the folders and open the readme.txt located there.

To run the code in this project the following libraries needs to be installed:
- OpenCV 	Version 3.4
- PCL 		Version 1.4
- RobWork
- RobWorkStudio

## Folders
<b>Integration</b> contains workcell and code used for combination of robotics and computer vision solution.
- Sources is located at: ROVIProjekt/Integration/Project_WorkCell/SamplePluginPA10/src/
- SamplePlugin.cpp and SamplePlugin.hpp contains code for combining the robotics and vision solution.

<b>MATLAB</b> contains scripts which were used for data analysis of different tests performed.

<b>ObjectFiles</b> contains some of the object files used in this project for the workcells in RobWorkStudio.

<b>Project_WorkCell_Cam</b> contains a workcell with the duck object in the scene.

<b>ROB</b> contains folders with sources and test for reachability analysis and RRT-connect.
- Reachability source is located at ROVIProjekt/ROB/Reachability/src/
- RRT-connect source is located at ROVIProjekt/ROB/rrt_connect/src/


<b>VIS</b> contains folders with sources and test for M2 and M3.
- M2 sources is located at ROVIProjekt/VIS/M2/project/src/
- M3 sources is located at ROVIProjekt/VIS/SparseStereo/src/

<b>interpolator</b> contains folders with sources and test for linear interpolation with and without parabolic blend.
- Linear interpolation (with and without parabolic blend) sources is located at ROVIProjekt/interpolator/linear/src/



## Source files

<b>Reachability</b>

<b>Linear interpolation (with and without parabolic blend)</b>

<b>RRT</b>

<b>M2</b>

<b>M3</b>

<b>Integration</b>
