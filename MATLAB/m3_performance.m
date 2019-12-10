clc; clear; format compact;

poses = importdata('/home/emil/Documents/M2ErrorEval/posesMATLAB.txt');
estimates = importdata('duck_performance.txt');
time = importdata('duck_performance_time.txt');

epsilon_t = ones(30,1);
epsilon_r = ones(30,1);

for i = 1:30
    a = 1 + (i - 1) * 4;
    b = 4 + (i - 1) * 4;
    pose = poses(a:b, 1:4); % Poses
    % Read corresponding pose estimate
    pose_estimate = estimates(a:b, 1:4);
    delta_P = pose_estimate(1:3,4) - pose(1:3,4);
    epsilon_t(i) = sqrt(sum(delta_P.^2));
    R = pose_estimate(1:3,1:3)*pose(1:3,1:3)';
    epsilon_r(i) = acos((trace(R)-1)/2);
end

disp('------------------------------------')
disp(' Mean errors ')
disp('------------------------------------')
mean_epsilon_t = mean(epsilon_t)
std_epsilon_t  = std(epsilon_t)
mean_epsilon_r = mean(epsilon_r)
std_epsilon_r  = std(epsilon_r)
disp('------------------------------------')
disp(' Mean time ')
disp('------------------------------------')
mean_time = mean(time)/1000 % ms
std_time  = std(time)/1000 % ms


found_indices = find(epsilon_t < 100);
size(found_indices,1)
mean_found_epsilon_t = mean(epsilon_t(found_indices))
std_found_epsilon_t = std(epsilon_t(found_indices))

mean_found_epsilon_r = mean(epsilon_r(found_indices))
std_found_epsilon_t = std(epsilon_r(found_indices))
%%

threshold_t = 0.05;
threshold_r = 0.785;

for i = 1:30
    a = 1 + (i - 1) * 4;
    b = 4 + (i - 1) * 4;
    pose = poses(a:b, 1:4); % Poses
    % Read corresponding pose estimate
    pose_estimate = estimates(a:b, 1:4);
    delta_P = pose_estimate(1:3,4) - pose(1:3,4);
    epsilon_t(i) = sqrt(sum(delta_P.^2));
    R = pose_estimate(1:3,1:3)*pose(1:3,1:3)';
    epsilon_r(i) = acos((trace(R)-1)/2);
end