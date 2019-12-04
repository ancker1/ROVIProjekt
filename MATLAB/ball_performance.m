clc;clear;close all;format compact;

T_World_Table = [eul2rotm([0 0 0]) [0; 0; -0.1;]; 0 0 0 1];
T_Table_World = inv(T_World_Table);

% Scanner
scanner_RPY = [0 0 deg2rad(-25)];
scanner_R = eul2rotm(scanner_RPY);
scanner_P = [0.05; 1.033; 0.8];
T_World_Scanner = [scanner_R scanner_P; 0 0 0 1];
estimates = load('ball_performance.txt');
estimates = estimates(1:(240 - 90),:)
poses = load('/home/mikkel/Desktop/Project_WorkCell_Cam/performEval_ball_pos.txt');
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