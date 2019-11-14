clc; clear; format compact;

% Pose from global alignment + ICP
T_Scanner_Obj = importdata('/home/emil/Documents/M2ErrorEval/combinedTransform.txt');

% Pose without ICP 
 T_Scanner_Obj_WorkAround = [0.998376 -0.0317489 0.0490396 -0.299058;
      0.046297 0.936031 -0.348861 -0.20763;
     -0.0332119 0.350485 0.935984 -0.880086;
      0 0 0 1]
 %T_Scanner_Obj = T_Scanner_Obj_WorkAround
  
% T_ICP = [0.998396 0.055124 -0.0132129 -0.000227558;
%                     -0.0548642 0.998308 0.0196348 -0.00330371;
%                      0.0142757 -0.0188768 0.999727 0.00351237;
%                      0 0 0 1]
 
T_World_Table = [eul2rotm([0 0 0]) [0; 0; -0.1;]; 0 0 0 1]
T_Table_World = inv(T_World_Table)

% Scanner
scanner_RPY = [0 0 deg2rad(-25)];
scanner_R = eul2rotm(scanner_RPY);
scanner_P = [0.05; 1.033; 0.8];
T_World_Scanner = [scanner_R scanner_P; 0 0 0 1]
 
% Object frame described in table frame 
T_Table_Obj = T_Table_World*T_World_Scanner*T_Scanner_Obj

Obj_RPY = rotm2eul(T_Table_Obj(1:3,1:3))
Obj_P = T_Table_Obj(1:3, 4)
 
% Ground truth frame
T_Table_GroundTruth = [eul2rotm([0 0 0]) [-0.25 0.474 0.188]'; 0 0 0 1]

%% Calculate errors
disp('--------------------------------------------------')
disp('Global Alignment: Positional error')
disp('--------------------------------------------------')
delta_P = T_Table_Obj(1:3,4) - T_Table_GroundTruth(1:3,4)
epsilon_t = sqrt(sum(delta_P.^2)) % Position error
disp('--------------------------------------------------')
disp('Global Alignment: Rotational error')
disp('--------------------------------------------------')
R = T_Table_Obj(1:3,1:3)*T_Table_GroundTruth(1:3,1:3)'
% trace(R) = 1 + 2*cos(theta)
epsilon_r = acos((trace(R)-1)/2)

% disp('--------------------------------------------------')
% disp('ICP: Positional error')
% disp('--------------------------------------------------')
% ICP_pose = T_Table_Obj*T_ICP
% delta_P = ICP_pose(1:3,4) - T_Table_GroundTruth(1:3,4)
% epsilon_t = sqrt(sum(delta_P.^2)) % Position error
% disp('--------------------------------------------------')
% disp('ICP: Rotational error')
% disp('--------------------------------------------------')
% R = ICP_pose(1:3,1:3)*T_Table_GroundTruth(1:3,1:3)'
% % trace(R) = 1 + 2*cos(theta)
% epsilon_r = acos((trace(R)-1)/2)

%% Comments
% Best error so far
% epsilon_t = 0.0018
% epsilon_t = 0.0988
% ICP: sqr_dist > 0.0001


%% Test with 30 random positions
clc; clear; format compact;
poses = importdata('/home/emil/Documents/M2ErrorEval/posesMATLAB.txt');
estimates = importdata('/home/emil/Documents/M2ErrorEval/voxel_1mm/poseEstTestCombined.txt');
time = importdata('/home/emil/Documents/M2ErrorEval/voxel_1mm/poseEstTestTime.txt');
time = time / 1000.0; % convert time to [s]
amount = importdata('/home/emil/Documents/M2ErrorEval/voxel_1mm/poseEstTestPoints.txt');

T_World_Table = [eul2rotm([0 0 0]) [0; 0; -0.1;]; 0 0 0 1];
T_Table_World = inv(T_World_Table);

% Scanner
scanner_RPY = [0 0 deg2rad(-25)];
scanner_R = eul2rotm(scanner_RPY);
scanner_P = [0.05; 1.033; 0.8];
T_World_Scanner = [scanner_R scanner_P; 0 0 0 1];
 
% Object frame described in table frame 
%T_Table_Obj = T_Table_World*T_World_Scanner*

epsilon_t = ones(30,1);
epsilon_r = ones(30,1);

for i = 1:30
    a = 1 + (i - 1) * 4;
    b = 4 + (i - 1) * 4;
    pose = poses(a:b, 1:4); % Poses
    % Read corresponding pose estimate
    pose_estimate = estimates(a:b, 1:4);
    pose_estimate = T_Table_World*T_World_Scanner*pose_estimate;
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
mean_time = mean(time)
std_time  = std(time)
disp('------------------------------------')
disp(' Mean points ')
disp('------------------------------------')
mean_points = mean(amount)
std_points  = std(amount)


% mean_epsilon_t =
%     0.0154
% std_epsilon_t =
%     0.0096
% mean_epsilon_r =
%     0.6498
% std_epsilon_r =
%     1.0385

%% Voxel leaf size test
clc;clear;close all;format compact;

T_World_Table = [eul2rotm([0 0 0]) [0; 0; -0.1;]; 0 0 0 1];
T_Table_World = inv(T_World_Table);

% Scanner
scanner_RPY = [0 0 deg2rad(-25)];
scanner_R = eul2rotm(scanner_RPY);
scanner_P = [0.05; 1.033; 0.8];
T_World_Scanner = [scanner_R scanner_P; 0 0 0 1];

all_epsilon_t = ones(30,10);
all_epsilon_r = ones(30,10);
all_points = ones(30,10);
all_times = ones(30,10);
poses = importdata('/home/emil/Documents/M2ErrorEval/posesMATLAB.txt');
start_path='/home/emil/Documents/M2ErrorEval/voxel_';
for i = 1:10
    estimates = importdata(strcat(start_path,int2str(i),'mm/poseEstTestCombined.txt'));
    time = importdata(strcat(start_path,int2str(i),'mm/poseEstTestTime.txt'));
    time = time / 1000.0; % convert time to [s]
    amount = importdata(strcat(start_path,int2str(i),'mm/poseEstTestPoints.txt'));
    for j = 1:30
        a = 1 + (j - 1) * 4;
        b = 4 + (j - 1) * 4;
        pose = poses(a:b, 1:4); % Poses
        % Read corresponding pose estimate
        pose_estimate = estimates(a:b, 1:4);
        pose_estimate = T_Table_World*T_World_Scanner*pose_estimate;
        delta_P = pose_estimate(1:3,4) - pose(1:3,4);
        epsilon_t(j) = sqrt(sum(delta_P.^2));
        R = pose_estimate(1:3,1:3)*pose(1:3,1:3)';
        epsilon_r(j) = acos((trace(R)-1)/2);
    end
    all_epsilon_t(:,i) = epsilon_t;
    all_epsilon_r(:,i) = epsilon_r;
    all_time(:,i) = time;
    all_points(:,i) = amount;
end
figure('Name','Epsilon_t')
boxplot(all_epsilon_t)
xlabel('Leaf size [mm]')
ylabel('\epsilon_t [m]')
set(gca,'FontSize',15)

figure('Name','Epsilon_r')
boxplot(all_epsilon_r)
xlabel('Leaf size [mm]')
ylabel('\epsilon_r [rad]')
set(gca,'FontSize',15)

figure('Name','Time')
boxplot(all_time)
xlabel('Leaf size [mm]')
ylabel('Time [s]')
set(gca,'FontSize',15)

figure('Name','Points')
boxplot(all_points)
xlabel('Leaf size [mm]')
ylabel('Amount of points')
set(gca,'FontSize',15)

%% Noise robustness test: varying variance
clc;clear;close all;format compact;

T_World_Table = [eul2rotm([0 0 0]) [0; 0; -0.1;]; 0 0 0 1];
T_Table_World = inv(T_World_Table);

% Scanner
scanner_RPY = [0 0 deg2rad(-25)];
scanner_R = eul2rotm(scanner_RPY);
scanner_P = [0.05; 1.033; 0.8];
T_World_Scanner = [scanner_R scanner_P; 0 0 0 1];
files = ['poseestimates_var0.000100.txt'; 
         'poseestimates_var0.000500.txt';
         'poseestimates_var0.001000.txt';
         'poseestimates_var0.005000.txt';
         'poseestimates_var0.010000.txt';
         'poseestimates_var0.050000.txt'];
poses = importdata('/home/emil/Documents/M2ErrorEval/posesMATLAB.txt');
all_epsilon_t = ones(30,size(files,1));
all_epsilon_r = ones(30,size(files,1));
for i = 1:size(files,1)
    estimates=importdata(strcat('/home/emil/Documents/M2ErrorEval/noisetest/',files(i,:))); 
    for j = 1:30
        a = 1 + (j - 1) * 4;
        b = 4 + (j - 1) * 4;
        pose = poses(a:b, 1:4); % Poses
        % Read corresponding pose estimate
        pose_estimate = estimates(a:b, 1:4);
        pose_estimate = T_Table_World*T_World_Scanner*pose_estimate;
        delta_P = pose_estimate(1:3,4) - pose(1:3,4);
        epsilon_t(j) = sqrt(sum(delta_P.^2));
        R = pose_estimate(1:3,1:3)*pose(1:3,1:3)';
        epsilon_r(j) = acos((trace(R)-1)/2);
    end
    all_epsilon_t(:,i) = epsilon_t;
    all_epsilon_r(:,i) = epsilon_r;
end
figure('Name','Epsilon_t')
boxplot(all_epsilon_t, [0.0001 0.0005 0.001 0.005 0.01 0.05])
xlabel('\sigma [m]')
ylabel('\epsilon_t [m]')
set(gca,'FontSize',15)

figure('Name','Epsilon_r')
boxplot(all_epsilon_r, [0.0001 0.0005 0.001 0.005 0.01 0.05])
xlabel('\sigma [m]')
ylabel('\epsilon_r [rad]')
set(gca,'FontSize',15)