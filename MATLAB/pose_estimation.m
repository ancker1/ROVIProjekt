clc; clear; format compact;

% Pose from global alignment + ICP
T_Scanner_Obj = importdata('/home/emil/Documents/M2ErrorEval/combinedTransform.txt');

% Pose without ICP 
 T_Scanner_Obj_Global = [0.998025 -0.039242 0.0490396 -0.300575;
      0.0541757 0.9329 -0.356037 -0.200923;
     -0.0317774 0.357991 0.933184 -0.882001;
      0 0 0 1]
% T_Scanner_Obj = T_Scanner_Obj_Global
  
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
poses = importdata('/home/emil/Documents/posesMATLAB.txt');
for i = 1:30
    a = 1 + (i - 1) * 4;
    b = 4 + (i - 1) * 4;
    poses(a:b, 1:4) % Poses
    % Read corresponding pose estimate
end


