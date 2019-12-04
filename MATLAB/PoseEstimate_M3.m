%% Calculating M3 pose seen from table
clc; clear; format compact;

% Pose from global alignment + ICP
T_CamL_Obj = importdata('M3_pose_estimate.txt');
rad2deg(rotm2eul(T_CamL_Obj(1:3,1:3)))
% T_rot_x = [eul2rotm([115 0 0]) [0; 0; 0;]; 0 0 0 1];
% T_rot_z = [eul2rotm([0 0 90]) [0; 0; 0;]; 0 0 0 1];
% T_CamL_Obj = T_CamL_Obj * T_rot_x;
% T_CamL_Obj = T_CamL_Obj * T_rot_z;

T_World_Table = [eul2rotm([0 0 0]) [0; 0; -0.1;]; 0 0 0 1];
T_Table_World = inv(T_World_Table);

% Left Cam
left_cam_RPY = [0 0 deg2rad(-25)];
left_cam_R = eul2rotm(left_cam_RPY);
left_cam_P = [-0.05; 1.033; 0.8];
T_World_CamL = [left_cam_R left_cam_P; 0 0 0 1];
 
% Object frame described in table frame 
T_Table_Obj = T_Table_World*T_World_CamL*T_CamL_Obj;


Obj_RPY = rotm2eul(T_Table_Obj(1:3,1:3))
Obj_P = T_Table_Obj(1:3, 4)


% 90+25 rundt om x og 90 rundt om z