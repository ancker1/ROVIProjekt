%% Plot for Transformation left
clc;clear;close all;format compact;

TPath_left = importdata('rrt_connect_data/rrtConnect_transform_pickplace_left.txt');
TPath_middle = importdata('rrt_connect_data/rrtConnect_transform_pickplace_middle.txt');
TPath_right = importdata('rrt_connect_data/rrtConnect_transform_pickplace_right.txt');



%figure('Name','Joint path')
%x = 1:nodes;
%plot(x,TPath_left)


% Inits
Px_mean = 0;
Py_mean = 0;
Pz_mean = 0;
roll_mean = 0;
pitch_mean = 0;
yaw_mean = 0;


for i = 1:num_of_iteration
    % Gets one iteration
    i = find(TPath_left(:,1)==(i-0))
    TPath_left_iteration = TPath_left(i,:);
    nodes=size(TPath_left_iteration,1);
    nodes=nodes/4;
    
    Px = 0;
    Py = 0;
    Pz = 0;
    roll = 0;
    pitch = 0;
    yaw = 0;
    for j = 1:nodes
        a = 1 + (j - 1) * 4;
        b = 4 + (j - 1) * 4;
        j
        TF = TPath_left_iteration(a:b, 1:4); % Poses
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
    Px_mean = Px_mean + Px;
    Py_mean = Py_mean + Py;
    Pz_mean = Pz_mean + Pz;
    roll_mean = roll_mean + roll;
    pitch_mean = pitch_mean + pitch;
    yaw_mean = yaw_mean + yaw;
end

Px_mean = Px_mean/num_of_iteration;
Py_mean = Py_mean/num_of_iteration;
Pz_mean = Pz_mean/num_of_iteration;
roll_mean = roll_mean/num_of_iteration;
pitch_mean = pitch_mean/num_of_iteration;
yaw_mean = yaw_mean/num_of_iteration;

figure('Name','Cartesian path')
plot(x,Px_mean,x,Py_mean,x,Pz_mean,'LineWidth',2)
legend('x', 'y', 'z')
set(gcf,'position',[0,0,1000*0.7,600*0.7])
xlabel('Step')
ylabel('Position in [m]')
set(gca,'FontSize',14)

figure('Name','RPY path')
plot(x,roll_mean,x,pitch_mean,x,yaw_mean,'LineWidth',2)
legend('R', 'P', 'Y')
set(gcf,'position',[0,0,1000*0.7,600*0.7])
xlabel('Step')
ylabel('Angle [rad]')
set(gca,'FontSize',14)

%figure('Name','Quaternion path')
%plot(x,q)

%% Path length: RPY Statistics
clc;clear;close all;format compact;

num_of_iteration = 60;

disp('                  Path 1                     ')
data = importdata('rrt_connect_data/rrtConnect_transform_pickplace_left.txt');
RRTPathXYZ_1 = pathXYZLength(data)
disp('                  Path 2                     ')
data = importdata('rrt_connect_data/rrtConnect_transform_pickplace_middle.txt');
RRTPathXYZ_2 = pathXYZLength(data)
disp('                  Path 3                     ')
data = importdata('rrt_connect_data/rrtConnect_transform_pickplace_right.txt');
RRTPathXYZ_3 = pathXYZLength(data)


disp('                  Path 1                     ')
data = importdata('rrt_connect_data/rrtConnect_transform_pickplace_left.txt');
for i = 1:num_of_iteration
    % Gets one iteration
    i = find(data(:,1)==(i-0));
    data_iteration = data(i,:);
    RRTPathRPY_1(i) = pathRPYLength(data);
end
    RRTPathRPY_1_mean = mean(RRTPathRPY_1)
    RRTPathRPY_1_std = std(RRTPathRPY_1)
disp('                  Path 2                     ')
data = importdata('rrt_connect_data/rrtConnect_transform_pickplace_middle.txt');
RRTPathRPY_2 = pathRPYLength(data)
disp('                  Path 3                     ')
data = importdata('rrt_connect_data/rrtConnect_transform_pickplace_right.txt');
RRTPathRPY_3 = pathRPYLength(data)

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
