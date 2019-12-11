%% Main information
clear; clc
number_of_repeated_data = 60;

%Make a plot of mean and std-dev of the data. Test if the choosen stepsize
%if the values is normal dist to say 3std-dev is equal to 99.7%(check this number) of all
%values is inside. 
%% RRT-connect stepsize vs configuration distance statistics

% Loading the configuration data 
dist_stepsize = load("rrt_connect_data/stepsize_vs_configuration_distance.txt");
[sorted I] = sort(dist_stepsize); % Sort by stepsizes 
dist_stepsize = [dist_stepsize(I(:,1),1) dist_stepsize(I(:,1),2)]; % Taking the indices of sorted
stepsize = dist_stepsize(:,1);
dist = dist_stepsize(:,2);


% mean and std_dev calculations:
mean_dist = zeros(size(dist_stepsize,1)/number_of_repeated_data, 2);
std_dev_dist = zeros(size(dist_stepsize,1)/number_of_repeated_data, 2);
for i = 0:(size(dist_stepsize,1)/number_of_repeated_data) - 1
    mean_dist(i+1,:) = mean(dist_stepsize(1+i*number_of_repeated_data:(i+1)*number_of_repeated_data,:)); % mean of stepsize should be = stepsize
    std_dev_dist(i+1,1) = dist_stepsize(1+i*number_of_repeated_data,1);
    std_dev_dist(i+1,2) = std(dist_stepsize(1+i*number_of_repeated_data:(i+1)*number_of_repeated_data,2));
end

% Plotting
figure('name', 'Configuration distance versus Stepsize')
subplot(4,1,1)
scatter(stepsize, dist)
title('Scatter plot')
xlabel('Stepsize')
ylabel('Configuration distance')

subplot(4,1,2)
plot(mean_dist(:,1), mean_dist(:,2))
title('Mean plot')
xlabel('Stepsize')
ylabel('Meaned configuration distance')

subplot(4,1,3)
plot(std_dev_dist(:,1), std_dev_dist(:,2))
title('Standard deviation plot')
xlabel('Stepsize')
ylabel('Standard deviation configuration distance')

hold on
figure('name', 'Boxplot of configuration distance versus stepsize')
boxplot(dist, stepsize);
title('Boxplot of configuration distance versus stepsize', 'FontSize', 30)
xlabel('Stepsize', 'FontSize', 20);
ylabel('Configuration distance', 'FontSize', 20)
set(findobj(gca,'type','line'),'linew',2)
set(gca,'FontSize',18)
xtickangle(45)
hold off
disp('-------------------------------------------------------------------')
disp('Finding minimum mean of configuration distance and corresponding stepsize')
disp('-------------------------------------------------------------------')
[minimum_mean, index] = min(mean_dist(:,2));
minimum_mean
stepsize_min_mean = mean_dist(index)

%% RRT-connect stepsize vs certesian distance statistics
% Distance is from world to TCP frame

% Loading the configuration data 
dist_stepsize = load("rrt_connect_data/stepsize_vs_cartesian_distance.txt");
[sorted I] = sort(dist_stepsize); % Sort by stepsizes 
dist_stepsize = [dist_stepsize(I(:,1),1) dist_stepsize(I(:,1),2)]; % Taking the indices of sorted
stepsize = dist_stepsize(:,1);
dist = dist_stepsize(:,2);


% mean and std_dev calculations:
mean_dist = zeros(size(dist_stepsize,1)/number_of_repeated_data, 2);
std_dev_dist = zeros(size(dist_stepsize,1)/number_of_repeated_data, 2);
for i = 0:(size(dist_stepsize,1)/number_of_repeated_data) - 1
    mean_dist(i+1,:) = mean(dist_stepsize(1+i*number_of_repeated_data:(i+1)*number_of_repeated_data,:)); % mean of stepsize should be = stepsize
    std_dev_dist(i+1,1) = dist_stepsize(1+i*number_of_repeated_data,1);
    std_dev_dist(i+1,2) = std(dist_stepsize(1+i*number_of_repeated_data:(i+1)*number_of_repeated_data,2));
end

% Plotting
figure('name', 'Cartersian distance for TCP versus Stepsize')
subplot(3,1,1)
scatter(stepsize, dist)
title('Scatter plot')
xlabel('Stepsize')
ylabel('Cartersian distance')

subplot(3,1,2)
plot(mean_dist(:,1), mean_dist(:,2))
title('Mean plot')
xlabel('Stepsize')
ylabel('Meaned Cartersian distance')

subplot(3,1,3)
plot(std_dev_dist(:,1), std_dev_dist(:,2))
title('Standard deviation plot')
xlabel('Stepsize')
ylabel('Standard deviation Cartersian distance')

hold on
figure('name', 'Boxplot of cartersian distance versus Stepsize')
boxplot(dist, stepsize);
xlabel('Stepsize')
ylabel('Cartersian distance')
set(findobj(gca,'type','line'),'linew',2)
set(gca,'FontSize',18)
xtickangle(45)
hold off

disp('-------------------------------------------------------------------')
disp('Finding minimum mean of cartersian distance and corresponding stepsize')
disp('-------------------------------------------------------------------')
[minimum_mean, index] = min(mean_dist(:,2));
minimum_mean
stepsize_min_mean = mean_dist(index)
%% RRT-connect stepsize vs calculation of path time statistics
% Loading the configuration data 
time_stepsize = load("rrt_connect_data/stepsize_vs_path_time.txt");
[sorted I] = sort(time_stepsize); % Sort by stepsizes 
time_stepsize = [time_stepsize(I(:,1),1) time_stepsize(I(:,1),2)]; % Taking the indices of sorted
stepsize = time_stepsize(:,1);
time = time_stepsize(:,2);

% mean and std_dev calculations:
mean_time = zeros(size(time_stepsize,1)/number_of_repeated_data, 2);
std_dev_time = zeros(size(time_stepsize,1)/number_of_repeated_data, 2);
for i = 0:(size(time_stepsize,1)/number_of_repeated_data) - 1
    mean_time(i+1,:) = mean(time_stepsize(1+i*number_of_repeated_data:(i+1)*number_of_repeated_data,:)); % mean of stepsize should be = stepsize
    std_dev_time(i+1,1) = time_stepsize(1+i*number_of_repeated_data,1);
    std_dev_time(i+1,2) = std(time_stepsize(1+i*number_of_repeated_data:(i+1)*number_of_repeated_data,2));
end

% Plotting
figure('name', 'Calculation time of path versus Stepsize')
subplot(3,1,1)
scatter(stepsize, time)
title('Scatter plot')
xlabel('Stepsize')
ylabel('Calculation time of path')

subplot(3,1,2)
plot(mean_time(:,1), mean_time(:,2))
title('Mean plot')
xlabel('Stepsize')
ylabel('Meaned calculation time of path')

subplot(3,1,3)
plot(std_dev_time(:,1), std_dev_time(:,2))
title('Standard deviation plot')
xlabel('Stepsize')
ylabel('Standard deviation calculation time of path')

hold on
figure('name', 'Boxplot of calculation time of path versus stepsize')
boxplot(time, stepsize);
title('Boxplot of calculation time of path versus stepsize', 'FontSize', 30)
xlabel('Stepsize', 'FontSize', 20)
ylabel('Calculation time of path [ms]', 'FontSize', 20)
set(findobj(gca,'type','line'),'linew',2)
set(gca,'FontSize',18)
xtickangle(45)
hold off

disp('-------------------------------------------------------------------')
disp('Finding minimum mean of time and corresponding stepsize')
disp('---------boxplot(time, stepsize);----------------------------------------------------------')
[minimum_mean, index] = min(mean_time(:,2));
minimum_mean
stepsize_min_mean = mean_time(index)

%% RRT-connect stepsize vs number of configuration statistics
% Loading the configuration data 
numconfig_stepsize = load("rrt_connect_data/stepsize_vs_configuration_number.txt");
[sorted I] = sort(numconfig_stepsize); % Sort by stepsizes 
numconfig_stepsize = [numconfig_stepsize(I(:,1),1) numconfig_stepsize(I(:,1),2)]; % Taking the indices of sorted
stepsize = numconfig_stepsize(:,1);
numconfig = numconfig_stepsize(:,2);

% mean and std_dev calculations:
mean_num_config = zeros(size(numconfig_stepsize,1)/number_of_repeated_data, 2);
std_dev_num_config = zeros(size(numconfig_stepsize,1)/number_of_repeated_data, 2);
for i = 0:(size(numconfig_stepsize,1)/number_of_repeated_data) - 1
    mean_num_config(i+1,:) = mean(numconfig_stepsize(1+i*number_of_repeated_data:(i+1)*number_of_repeated_data,:)); % mean of stepsize should be = stepsize
    std_dev_num_config(i+1,1) = numconfig_stepsize(1+i*number_of_repeated_data,1);
    std_dev_num_config(i+1,2) = std(numconfig_stepsize(1+i*number_of_repeated_data:(i+1)*number_of_repeated_data,2));
end

% Plotting
figure('name', 'Number of configuration versus Stepsize')
subplot(3,1,1)
scatter(stepsize, numconfig)
title('Scatter plot')
xlabel('Stepsize')
ylabel('Number of configuration ')

subplot(3,1,2)
plot(mean_num_config(:,1), mean_num_config(:,2))
title('Mean plot')
xlabel('Stepsize')
ylabel('Meaned number of configuration ')

subplot(3,1,3)
plot(std_dev_num_config(:,1), std_dev_num_config(:,2))
title('Standard deviation plot')
xlabel('Stepsize')
ylabel('Standard deviation number of configuration ')

hold on
figure('name', 'Boxplot of number of configuration versus Stepsize')
boxplot(numconfig, stepsize);
title('Boxplot of number of configuration versus stepsize', 'FontSize', 30)
xlabel('Stepsize', 'FontSize', 20)
ylabel('Number of configuration ', 'FontSize', 20)
set(findobj(gca,'type','line'),'linew',2)
set(gca,'FontSize',18)
xtickangle(45)
hold off

disp('-------------------------------------------------------------------')
disp('Finding minimum mean of number of configuration and corresponding stepsize')
disp('-------------------------------------------------------------------')
[minimum_mean, index] = min(mean_num_config(:,2));
minimum_mean
stepsize_min_mean = mean_num_config(index)
