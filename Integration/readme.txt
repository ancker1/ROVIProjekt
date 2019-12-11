All files used for integration is placed within: /Project_WorkCell
All source files used for integration is placed within: /Project_WorkCell/SamplePluginPA10/src

************************************************
*	     To run integration 	       *
************************************************
1) Build the plugin
$ cd /Project_WorkCell/SamplePluginPA10/build
$ cmake ..
$ make

2) Open RobWorkStudio while standing in folder: /Project_WorkCell/SamplePluginPA10/libs/Release
3) Load plugin:
	/Project_WorkCell/SamplePluginPA10/libs/Release/libRoViPluginPA10.so

OBS: if the error "buffer overflow detected" is present, then this might be caused by the path to the workcell is too long. 
Try moving the workcell or whole folder to a place with a shorter path (remember to edit absolute paths).

************************************************
*	     Using the plugin   	       *
************************************************
Press the buttons in the GUI in RobWorkStudio, when plugin have been loaded.
