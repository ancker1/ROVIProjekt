Folder overview
- Reachability: contains work that was done to develop vision method M1.
- rrt_connect: contains work done to develop vision method M2.


**********************************************
*	    Running Reachability tests	     *
**********************************************
Building and running main.cpp directly will perform a pose estimate on an example.


To run the tests open the main.cpp file
Uncomment lines from 350-370 depending on the tests
- 351: robustness test (varying variance test)
- 355 & 356: varying voxel leaf size in preprocessing of scene
- 360 & 361: time test of RANSAC
- 365: generation of poses
- 369: fetch ground truth


OBS! if tests is being run; edit absolute paths at the following lines (main.cpp).
- 133
- 149
- 201
- 202
- 231
- 263
- 320



**********************************************
*	   Running rrt_connect tests	     *
**********************************************
Both testing method and evaluation method is included in main.cpp.
The main functionen is descripted below.
- (Visualization by LUA file) line 56-76
- (Generating data to boxplot of different stepsizes) line 82-112
- (Generating data with optimal stepsize at 3 different pick place) line 119-237

All the methods needs path to the workcell which have the bin around the pick area and the cylinder.
Thus modify the path below to the supplied workcell "ROVIProjekt/Project_WorkCell_Cam_RRT".
- line 22 



