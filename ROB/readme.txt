Folder overview
- Reachability: contains work that was done to develop vision method M1.
- rrt_connect: contains work done to develop vision method M2.


**********************************************
*	    Running Reachability tests		     *
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
*	    Running rrt_connect tests		     *
**********************************************
Two methods were implemented for sparse stereo. 

3D Pose estimation (yellow ball):
- (Single test) Outcomment line 83-88
- (Performance evaluation) Outcomment line 92-98

6D Pose estimation (dotted duck):
- (Single test) Outcomment line 103-111
- (Performance evaluation) Outcomment line 114-120

Both methods need pictures. Thus modify the paths.
- 68 (get the camera matrix from supplied workcell)
- 84
- 85
- 94
- 105
- 106
- 107
- 108
- 116


