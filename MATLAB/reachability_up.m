%% Figure 1
clc; clear; close all;

% 10 deg in yaw from above
% data_place = importdata('up/base_pos_up_place.txt');
% data_pick_left = importdata('up/base_pos_up_pick_left.txt');
% data_pick_mid = importdata('up/base_pos_up_pick_mid.txt');
% data_pick_right = importdata('up/base_pos_up_pick_right.txt');

% Directly above
data_place = importdata('up/base_pos_directly_up_place.txt');
data_pick_left = importdata('up/base_pos_directly_up_pick_left.txt');
data_pick_mid = importdata('up/base_pos_directly_up_pick_mid.txt');
data_pick_right = importdata('up/base_pos_directly_up_pick_right.txt');

data = data_place;
data(:,3) = data_place(:,3) + data_pick_left(:,3) + data_pick_mid(:,3) + data_pick_right(:,3);
%data(:,3) = data_place(:,3);

tablex = [-40 40 40 -40];
tabley = [-60 -60 60 60];

pickx = [-40 40 40 -40];
picky = [-60 -60 -30 -30];

placex = [-40 -20 -20 -40];
placey = [40 40 60 60];

colormap default
cmap = parula(max(data(:,3))+1);

%figure
figure(1)
fill(tablex,tabley,[0.9 0.9 0.9])
hold on
fill(pickx, picky, [0.9 0.9 0])
fill(placex, placey, [0 0.8 0])
sz = 175;

c = cmap(data(:,3)+1,:);

scatter(-100*data(:,1), -100*data(:,2),sz,c,'filled')
scatter(-100*[-0.25],-100*[0.474],400,[0.50 0.50 0.50], 'filled')
scatter(-100*[0],-100*[0.474],400,[0.50 0.50 0.50], 'filled')
scatter(-100*[0.25],-100*[0.474],400,[0.50 0.50 0.50], 'filled')
scatter(-100*[0.30],100*[0.5],400,[0.50 0.50 0.50], 'filled')

colorbar

text(-8, -35, 'Pick area');
text(-40,63,'Place area')
%text(-37,48,'area')

xlim([-80 80])
ylim([-80 80])
hold off

%% (MIN-MAX) Normalized data
clc; clear;

% 10 deg in yaw from above
% data_place = importdata('up/base_pos_up_place.txt');
% data_pick_left = importdata('up/base_pos_up_pick_left.txt');
% data_pick_mid = importdata('up/base_pos_up_pick_mid.txt');
% data_pick_right = importdata('up/base_pos_up_pick_right.txt');

% Directly above
data_place = importdata('up/base_pos_directly_up_place.txt');
data_pick_left = importdata('up/base_pos_directly_up_pick_left.txt');
data_pick_mid = importdata('up/base_pos_directly_up_pick_mid.txt');
data_pick_right = importdata('up/base_pos_directly_up_pick_right.txt');

data = data_place;
%data(:,3) = data_place(:,3) + data_pick_left(:,3) + data_pick_mid(:,3) + data_pick_right(:,3);
data(:,3) = data_pick_right(:,3);
mdata = [1000*mat2gray(data(:,3),[min(data(:,3)) max(data(:,3))])];
mdata = [data(:,1) data(:,2) mdata];

tablex = [-40 40 40 -40];
tabley = [-60 -60 60 60];

pickx = [-40 40 40 -40];
picky = [-60 -60 -30 -30];

placex = [-40 -20 -20 -40];
placey = [40 40 60 60];

colormap default
cmap = parula(max(round(mdata(:,3)))+1);

%figure
figure(2)
fill(tablex,tabley,[0.9 0.9 0.9])
hold on
fill(pickx, picky, [0.9 0.9 0])
fill(placex, placey, [0 0.8 0])
sz = 75;

c = cmap(round(mdata(:,3))+1,:);

scatter(-100*mdata(:,1), -100*mdata(:,2),sz,c,'filled')
scatter(-100*[-0.25],-100*[0.474],400,[0.50 0.50 0.50], 'filled')
%scatter(-100*[0],-100*[0.474],400,[0.50 0.50 0.50], 'filled')
%scatter(-100*[0.25],-100*[0.474],400,[0.50 0.50 0.50], 'filled')
%scatter(-100*[0.30],100*[0.5],400,[0.50 0.50 0.50], 'filled')

colorbar

text(-8, -35, 'Pick area');
text(-40,63,'Place area')
%text(-37,48,'area')

xlim([-80 80])
ylim([-80 80])
hold off

%% Surface plot
clc; clear; close all;

% 10 deg in yaw from above
% data_place = importdata('up/base_pos_up_place.txt');
% data_pick_left = importdata('up/base_pos_up_pick_left.txt');
% data_pick_mid = importdata('up/base_pos_up_pick_mid.txt');
% data_pick_right = importdata('up/base_pos_up_pick_right.txt');

% Directly above
data_place = importdata('up/base_pos_directly_up_place.txt');
data_pick_left = importdata('up/base_pos_directly_up_pick_left.txt');
data_pick_mid = importdata('up/base_pos_directly_up_pick_mid.txt');
data_pick_right = importdata('up/base_pos_directly_up_pick_right.txt');

% data = data_place
% data = data_pick_left
% data = data_pick_mid
% data = data_pick_right
data = data_pick_left + data_pick_mid + data_pick_right + data_place*3; % do not multiply with 3 if not using equal weighting

x = data(:,1)./6; % ./4 if not using equal weighting
y = data(:,2)./6; % ./4 if not using equal weighting
z = data(:,3);

z = mat2gray(z,[min(z) max(z)]);

figure(2)
stem3(x,y,z)
grid on

xv = linspace(min(x), max(x), 20);
yv = linspace(min(y), max(y), 20);
[X,Y] = meshgrid(xv, yv);
Z = griddata(x,y,z,X,Y);

xv = linspace(-0.4, 0.4, 20);
yv = linspace(-0.6, 0.6, 20);
[X2,Y2] = meshgrid(xv, yv);
Z2 = zeros(20,20);
C2 = ones(20,20,3).*0.9;

xv = linspace(-0.4, 0.4, 20);
yv = linspace(-0.6, -0.3, 20);
[X3,Y3] = meshgrid(xv, yv);
Z3 = zeros(20,20);
C3 = ones(20,20,3).*0.4;

xv = linspace(-0.4, -0.2, 20);
yv = linspace(0.4, 0.6, 20);
[X4,Y4] = meshgrid(xv, yv);
Z4 = zeros(20,20);
C4 = ones(20,20,3).*0.4;

figure('Name','Reachability Analysis','Position', [10 10 900 600])

surf(X, Y, Z);
hold on
surf(X2,Y2,Z2,C2) % Table
surf(X3,-Y3,Z3,C3) % Pick
surf(-X4,-Y4,Z4,C4) % Place
scatter([-0.25],[0.474],1200,[0.3 0.9 0.3], 'filled')
scatter([0.00],[0.474],1200,[0.3 0.9 0.3], 'filled')
scatter([0.25],[0.474],1200,[0.3 0.9 0.3], 'filled')
scatter([0.30],-[0.500],1200,[0.3 0.9 0.3], 'filled')
scatter3([-0.0081], [-0.0403], [2], 400, [1 0 0], 'filled'); % Drawing optimal place from side test
hold off
colorbar
xlim([-0.6 0.5])
ylim([-0.5 0.5])
grid on
text(-0.4, 0.585, 'Pick area', 'Color', 'white');
text(0.2,-0.415,'Place area', 'Color', 'white')
shading interp
axis equal
view(0, 90); % azimuth elevation
%title('Reachability Analysis')

% Many maximums
%base_pos_indx = find(max(z)==z)
%base_pos = [x(base_pos_indx) y(base_pos_indx) 0.150]