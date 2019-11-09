%% RRT-connect stepsize vs configuration distance statistics

% Loading the configuration data 
dist_stepsize = load("build-rrt_connect-Desktop-Release/stepsize_vs_configuration_distance.txt");
stepsize = dist_stepsize(:,1);
dist = dist_stepsize(:,2);

% Plotting
figure('name', 'Configuration distance versus Stepsize')
scatter(stepsize, dist)

%% RRT-connect stepsize vs certesian distance statistics
% Distance is from world to TCP frame

% Loading the configuration data 
dist_stepsize = load("build-rrt_connect-Desktop-Release/stepsize_vs_cartesian_distance.txt");
stepsize = dist_stepsize(:,1);
dist = dist_stepsize(:,2);

% Plotting
figure('name', 'Cartersian distance versus Stepsize')
scatter(stepsize, dist)

%% RRT-connect stepsize vs calculation of path time statistics
% Loading the configuration data 
time_stepsize = load("build-rrt_connect-Desktop-Release/stepsize_vs_path_time.txt");
stepsize = time_stepsize(:,1);
time = time_stepsize(:,2);

% Plotting
figure('name', 'Calculation time of path versus Stepsize')
scatter(stepsize, time)

%% RRT-connect stepsize vs number of configuration statistics
% Loading the configuration data 
numconfig_stepsize = load("build-rrt_connect-Desktop-Release/stepsize_vs_configuration_number.txt");
stepsize = numconfig_stepsize(:,1);
numconfig = numconfig_stepsize(:,2);

% Plotting
figure('name', 'Number of configurations versus Stepsize')
scatter(stepsize, numconfig)
