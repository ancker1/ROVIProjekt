%% Plot for Transformation left
clc;clear;close all;format compact;

TPath_left = importdata('rrt_connect_data/rrtConnect_transform_pickplace_left.txt');
TPath_middle = importdata('rrt_connect_data/rrtConnect_transform_pickplace_middle.txt');
TPath_right = importdata('rrt_connect_data/rrtConnect_transform_pickplace_right.txt');

%Plot the first iteration
i = find(TPath_left(:,1)==(0));
TPath_left = TPath_left(i,:);
TPath_left(:,1) = [];% deletes first row because only descibes index

nodes = size(TPath_left,1)/4;

%figure('Name','Joint path')
x = 1:nodes;
%plot(x,TPath_left)


for j = 1:nodes
    a = 1 + (j - 1) * 4;
    b = 4 + (j - 1) * 4;
    
    TF = TPath_left(a:b, 1:4); % Poses
    Px(j) = TF(1,4);
    Py(j) = TF(2,4);
    Pz(j) = TF(3,4);
    R = TF(1:3,1:3);

    RPY = rotm2eul(R);
    roll(j) = RPY(1);
    pitch(j) = RPY(2);
    yaw(j) = RPY(3);
    if yaw(j) > 3
        yaw(j) = -3.142;
    end
end


figure('Name','Cartesian path')
plot(x,Px,x,Py,x,Pz,'LineWidth',2)
legend('x', 'y', 'z')
set(gcf,'position',[0,0,1000*0.7,600*0.7])
xlabel('Step')
ylabel('Position in [m]')
set(gca,'FontSize',14)

figure('Name','RPY path')
plot(x,roll,x,pitch,x,yaw,'LineWidth',2)
legend('R', 'P', 'Y')
set(gcf,'position',[0,0,1000*0.7,600*0.7])
xlabel('Step')
ylabel('Angle [rad]')
set(gca,'FontSize',14)

%figure('Name','Quaternion path')
%plot(x,q)

%% Path planning time
clc;clear;close all;format compact;

disp('                  Path 1                     ')
data = importdata('rrt_connect_data/rrtConnect_planning_times_pickplace_left.txt');
path_time_mean_1 = mean(data)
path_time_std_1 = std(data)

disp('                  Path 2                     ')
data = importdata('rrt_connect_data/rrtConnect_planning_times_pickplace_middle.txt');
path_time_mean_2 = mean(data)
path_time_std_2 = std(data)

disp('                  Path 3                     ')
data = importdata('rrt_connect_data/rrtConnect_planning_times_pickplace_right.txt');
path_time_mean_3 = mean(data)
path_time_std_3 = std(data)

%% Path length: RPY Statistics
clc;clear;close all;format compact;

num_of_iteration = 60;

disp('                  Path 1                     ')
data = importdata('rrt_connect_data/rrtConnect_transform_pickplace_left.txt');
for i = 1:num_of_iteration
    % Gets one iteration
    Index = find(data(:,1)==(i-1));
    data_iteration = data(Index,:);
    data_iteration(:,1) = [];% deletes first row because only descibes index
    RRTPathXYZ_1(i) = pathXYZLength(data_iteration);
end
RRTPathXYZ_1_mean = mean(RRTPathXYZ_1)
RRTPathXYZ_1_std = std(RRTPathXYZ_1)

disp('                  Path 2                     ')
data = importdata('rrt_connect_data/rrtConnect_transform_pickplace_middle.txt');
for i = 1:num_of_iteration
    % Gets one iteration
    Index = find(data(:,1)==(i-1));
    data_iteration = data(Index,:);
    data_iteration(:,1) = [];% deletes first row because only descibes index
    RRTPathXYZ_2(i) = pathXYZLength(data_iteration);
end
RRTPathXYZ_2_mean = mean(RRTPathXYZ_2)
RRTPathXYZ_2_std = std(RRTPathXYZ_2)

disp('                  Path 3                     ')
data = importdata('rrt_connect_data/rrtConnect_transform_pickplace_right.txt');
for i = 1:num_of_iteration
    % Gets one iteration
    Index = find(data(:,1)==(i-1));
    data_iteration = data(Index,:);
    data_iteration(:,1) = [];% deletes first row because only descibes index
    RRTPathXYZ_1(i) = pathXYZLength(data_iteration);
end
RRTPathXYZ_1_mean = mean(RRTPathXYZ_1)
RRTPathXYZ_1_std = std(RRTPathXYZ_1)


disp('                  Path 1                     ')
data = importdata('rrt_connect_data/rrtConnect_transform_pickplace_left.txt');
for i = 1:num_of_iteration
    % Gets one iteration
    Index = find(data(:,1)==(i-1));
    data_iteration = data(Index,:);
    data_iteration(:,1) = [];% deletes first row because only descibes index
    RRTPathRPY_3(i) = pathRPYLength(data_iteration);
end
RRTPathRPY_3_mean = mean(RRTPathRPY_3)
RRTPathRPY_3_std = std(RRTPathRPY_3)

disp('                  Path 2                     ')
data = importdata('rrt_connect_data/rrtConnect_transform_pickplace_middle.txt');
for i = 1:num_of_iteration
    % Gets one iteration
    Index = find(data(:,1)==(i-1));
    data_iteration = data(Index,:);
    data_iteration(:,1) = [];% deletes first row because only descibes index
    RRTPathRPY_2(i) = pathRPYLength(data_iteration);
end
RRTPathRPY_2_mean = mean(RRTPathRPY_2)
RRTPathRPY_2_std = std(RRTPathRPY_2)

disp('                  Path 3                     ')
data = importdata('rrt_connect_data/rrtConnect_transform_pickplace_right.txt');
for i = 1:num_of_iteration
    % Gets one iteration
    Index = find(data(:,1)==(i-1));
    data_iteration = data(Index,:);
    data_iteration(:,1) = [];% deletes first row because only descibes index
    RRTPathRPY_3(i) = pathRPYLength(data_iteration);
end
RRTPathRPY_3_mean = mean(RRTPathRPY_3)
RRTPathRPY_3_std = std(RRTPathRPY_3)

function RPYlength = pathRPYLength(data)
    nodes = size(data,1)/4;
    for j = 1:nodes
        a = 1 + (j - 1) * 4;
        b = 4 + (j - 1) * 4;
        TF = data(a:b, 1:4); % Poses
        
        R = TF(1:3,1:3);
    
        RPY = rotm2eul(R);
        roll(j) = RPY(1);
        pitch(j) = RPY(2);
        yaw(j) = RPY(3);
        if yaw(j) > 3
            yaw(j) = -3.142;
        end
    end
    RPYlength = sum(sqrt(diff(roll).^2 + diff(pitch).^2 + diff(yaw).^2));
end


function XYZlength = pathXYZLength(data)
    nodes = size(data,1)/4;
    for j = 1:nodes
        a = 1 + (j - 1) * 4;
        b = 4 + (j - 1) * 4;
        TF = data(a:b, 1:4); % Poses
        Px(j) = TF(1,4);
        Py(j) = TF(2,4);
        Pz(j) = TF(3,4);
        R = TF(1:3,1:3);
    end
    XYZlength = sum(sqrt(diff(Px).^2 + diff(Py).^2 + diff(Pz).^2));
end