Folder overview
- Reachability: contains work that was done to develop reachability.
- rrt_connect: contains work done to develop trajectory planning via rrt-connect.


**********************************************
*	    Running Reachability tests	     *
**********************************************
Both a visulization method and evaluation for grasping from top and side is included in main.cpp.
To run the script edit the path in line 209 of the workcell to the supplied workcell "ROVIProjekt/Project_WorkCell_Cam_RRT/Project_WorkCell/Scene.wc.xml".
Futhermore in the edit the scene file so only the table and cylinder is present.

The methods is is descripted below.
- (Reachability for grasping above data generation) line 243-272
- (Reachability for grasping side data generation) line 278-305 
- (Visualization by visu.rwplay) line 310-330


**********************************************
*	   Running rrt_connect tests	     *
**********************************************
Both testing method and evaluation method is included in main.cpp.
The main function is descripted below.
- (Visualization by LUA file) line 56-76
- (Generating data to boxplot of different stepsizes) line 82-112
- (Generating data with optimal stepsize at 3 different pick place) line 119-237

All the methods needs path to the workcell which have the bin around the pick area and the cylinder.
Thus modify the path below to the supplied workcell "ROVIProjekt/Project_WorkCell_Cam_RRT/Project_WorkCell/Scene.wc.xml".
- line 22 



