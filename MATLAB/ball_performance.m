clc;clear;close all;format compact;

estimates = load('ball_performance.txt');
estimates = estimates(1:(240 - 90),:);
poses = load('/home/mikkel/Desktop/Project_WorkCell_Cam/performEval_ball_pos.txt');
poses = poses(1:30,:);
all_epsilon_t = ones(30, size(estimates,1)/30);
epsilon_t = ones(30, 1);
for i = 1:size(estimates,1)/30
    offset = (i - 1)*30
    for j = 1:30
        pose_estimate = estimates(j + offset, 2:4);
        pose = poses(j, 1:3);
        delta_P = pose_estimate - pose;
        epsilon_t(j) = sqrt(sum(delta_P.^2));
    end
    all_epsilon_t(:,i) = epsilon_t;
end

error_levels = [0 1 5 20 50];

figure('Name','Epsilon_t')
boxplot(all_epsilon_t, error_levels)
xlabel('\sigma [RGB intensity]')
ylabel('\epsilon_t [m]')
set(gca,'FontSize',15)
set(gcf,'position',[0,0,700,400])

%%

estimates = load('ball_performance.txt');
poses = load('/home/mikkel/Desktop/Project_WorkCell_Cam/performEval_ball_pos.txt');
poses = poses(1:30,:);
all_epsilon_t = ones(30, size(estimates,1)/30);
epsilon_t = ones(30, 1);
for i = 1:size(estimates,1)/30
    offset = (i - 1)*30
    for j = 1:30
        pose_estimate = estimates(j + offset, 2:4);
        pose = poses(j, 1:3);
        delta_P = pose_estimate - pose;
        epsilon_t(j) = sqrt(sum(delta_P.^2));
    end
    all_epsilon_t(:,i) = epsilon_t;
end

error_levels = [0 1 5 20 50 100 200 255];

figure('Name', 'Performance: Translation')
T_t = 0.05;
T_r = deg2rad(45);
for i = 1:size(estimates,1)/30
    performance_t(i) = size(find(all_epsilon_t(:,i) <= T_t),1)/size(all_epsilon_t,1);
end
plot(error_levels, performance_t)
xticks([0 20 50 100 200 255])
legend('Translation')
xlabel('\sigma [RGB intensity]')
ylabel('Success ratio')
set(gca,'FontSize',15)
set(gcf,'position',[0,0,900,400])

%% Time performance
times = load('ball_performance_time.txt');
mean_times = mean(times)/10^3 % ms
mean_times_std = std(times)/10^3 % ms

%% Without any noise
clc;clear;
 
disp('TIME PERFORMANCE')
times = load('ball_performance_time_without_noise.txt');
mean_times = mean(times)/10^3 % ms
mean_times_std = std(times)/10^3 % ms

disp('POSE ESTIMATION ERROR')
poses = load('/home/mikkel/Desktop/Project_WorkCell_Cam/performEval_ball_pos.txt');
poses = poses(1:30,:);
pose_estimates = load('ball_performance_without_noise.txt');
pose_estimates(:,1) = []; % Removing 1 col

pos_dif = pose_estimates - poses;
a = sqrt(sum(pos_dif.^2,2));
mean_pos_dif = mean(a)
std_pos_dif = std(a)
