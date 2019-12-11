To run the interpolation tests

1) comment in/out the moveTo that is needed to decide which pick location to use at line 70,71,72.
2) build and run linear.cpp

Files will be created containing relevant data.


OBS if error "buffer overflow detected" is present, then the path to the project is too long.
- Edit the path at line 30 to where the workcell is placed. This is written as absolute to avoid "buffer overflow detected", which is an error caused by RobWorkStudio.
