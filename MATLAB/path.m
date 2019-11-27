clc;clear;close all;format compact;
QPath=importdata('/home/emil/Documents/LinIntQ.txt');
CPath=importdata('/home/emil/Documents/LinIntTF.txt');

blendData=importdata('/home/emil/Documents/blend.txt');

nodes=size(QPath,1);

figure('Name','Joint path')
x = 1:nodes;
plot(x,QPath)



for j = 1:nodes
    a = 1 + (j - 1) * 4;
    b = 4 + (j - 1) * 4;
    TF = CPath(a:b, 1:4); % Poses
    Px(j) = TF(1,4);
    Py(j) = TF(2,4);
    Pz(j) = TF(3,4);
    R = TF(1:3,1:3);
    
    RPY = rotm2eul(R);
    roll(j) = RPY(1);
    pitch(j) = RPY(2);
    yaw(j) = RPY(3);
    if yaw(j) > 3
        yaw(j) = min(yaw);
    end
      
    
    quatern = rotm2quat(R);
    q(1,j) = quatern(1);
    q(2,j) = quatern(2);
    q(3,j) = quatern(3);
    q(4,j) = quatern(4);

end

nodes=size(blendData,1);
for j = 1:nodes
    a = 1 + (j - 1) * 4;
    b = 4 + (j - 1) * 4;
    PxBlend(j) = blendData(j,1);
    PyBlend(j) = blendData(j,2);
    PzBlend(j) = blendData(j,3);
end

xblend = linspace(1,size(QPath,1),size(blendData,1));

figure('Name','Cartesian path')
plot(x,Px,x,Py,x,Pz,xblend,PxBlend,'--',xblend,PyBlend,'--',xblend,PzBlend,'--')
legend('x', 'y', 'z')
figure('Name','RPY path')
plot(x,roll,x,pitch,x,yaw)
legend('R', 'P', 'Y')
figure('Name','Quaternion path')
plot(x,q)

%% v1

clc;clear;close all;format compact;
QPath=importdata('/home/emil/Documents/LinIntQ.txt');
CPath=importdata('/home/emil/Documents/LinIntTF.txt');

blendData=importdata('/home/emil/Documents/blend_v1.txt');

nodes=size(QPath,1);

figure('Name','Joint path')
x = 1:nodes;
plot(x,QPath)



for j = 1:nodes
    a = 1 + (j - 1) * 4;
    b = 4 + (j - 1) * 4;
    TF = CPath(a:b, 1:4); % Poses
    Px(j) = TF(1,4);
    Py(j) = TF(2,4);
    Pz(j) = TF(3,4);
    R = TF(1:3,1:3);
    
    RPY = rotm2eul(R);
    roll(j) = RPY(1);
    pitch(j) = RPY(2);
    yaw(j) = RPY(3);
    if yaw(j) > 3
        yaw(j) = min(yaw);
    end
      
    
    quatern = rotm2quat(R);
    q(1,j) = quatern(1);
    q(2,j) = quatern(2);
    q(3,j) = quatern(3);
    q(4,j) = quatern(4);

end

nodes=size(blendData,1)/4;
for j = 1:nodes
    a = 1 + (j - 1) * 4;
    b = 4 + (j - 1) * 4;
    TF = blendData(a:b,1:4);
    PxBlend(j) = TF(1,4);
    PyBlend(j) = TF(2,4);
    PzBlend(j) = TF(3,4);
    R = TF(1:3,1:3);
end

xblend = linspace(1,size(QPath,1),size(blendData,1)/4);

figure('Name','Cartesian path')
plot(x,Px,x,Py,x,Pz,xblend,PxBlend,'--',xblend,PyBlend,'--',xblend,PzBlend,'--')
legend('x', 'y', 'z')
figure('Name','RPY path')
plot(x,roll,x,pitch,x,yaw)
legend('R', 'P', 'Y')
figure('Name','Quaternion path')
plot(x,q)





