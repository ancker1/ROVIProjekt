clc;clear;close all;format compact;
%% Execution Time

linear = zeros(60,3);
linear(:,1) = importdata('interpolation/LIexetime_micros_1.txt') * 10^-3;
linear(:,2) = importdata('interpolation/LIexetime_micros_2.txt') * 10^-3;
linear(:,3) = importdata('interpolation/LIexetime_micros_3.txt') * 10^-3;

parabolic = zeros(60,3);
parabolic(:,1) = importdata('interpolation/PBexetime_micros_1.txt') * 10^-3;
parabolic(:,2) = importdata('interpolation/PBexetime_micros_2.txt') * 10^-3;
parabolic(:,3) = importdata('interpolation/PBexetime_micros_3.txt') * 10^-3;

mean_linear = mean(linear)
std_linear = std(linear) 

mean_parabolic = mean(parabolic)
std_parabolic = std(parabolic)

%% Path length: RPY
clc;clear;close all;format compact;


disp('                  Path 1                     ')
data = importdata('interpolation/LinIntTF_1.txt');
linearPathXYZ_1 = pathXYZLength(data)
data = importdata('interpolation/blend_tau25_1.txt');
parabolicPathXYZ_1 = pathXYZLength(data)
disp('                  Path 2                     ')
data = importdata('interpolation/LinIntTF_2.txt');
linearPathXYZ_2 = pathXYZLength(data)
data = importdata('interpolation/blend_tau25_2.txt');
parabolicPathXYZ_2 = pathXYZLength(data)
disp('                  Path 3                     ')
data = importdata('interpolation/LinIntTF_3.txt');
linearPathXYZ_3 = pathXYZLength(data)
data = importdata('interpolation/blend_tau25_3.txt');
parabolicPathXYZ_3 = pathXYZLength(data)



disp('                  Path 1                     ')
data = importdata('interpolation/LinIntTF_1.txt');
linearPathRPY_1 = pathRPYLength(data)
data = importdata('interpolation/blend_tau25_1.txt');
parabolicPathRPY_1 = pathRPYLength(data)
disp('                  Path 2                     ')
data = importdata('interpolation/LinIntTF_2.txt');
linearPathRPY_2 = pathRPYLength(data)
data = importdata('interpolation/blend_tau25_2.txt');
parabolicPathRPY_2 = pathRPYLength(data)
disp('                  Path 3                     ')
data = importdata('interpolation/LinIntTF_3.txt');
linearPathRPY_3 = pathRPYLength(data)
data = importdata('interpolation/blend_tau25_3.txt');
parabolicPathRPY_3 = pathRPYLength(data)



%% Plot of trajectory in join space (interpolation performed in Cartesian space)
clc; clear; close all; format compact;
data = importdata('interpolation/LinIntQ_1.txt');
length = size(data,1);
x = 1:length;
plot(x, data)
legend('q_0', 'q_1', 'q_2', 'q_3', 'q_4', 'q_5')
xlabel('Step')
ylabel('Joint value [rad]')
set(gcf,'position',[0,0,800,400])
set(gca,'FontSize',14)

%% Function definitions
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