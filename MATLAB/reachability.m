%% Figure 1
clc; clear; close all;

data_place = importdata('side/base_pos_side_place.txt');
data_pick_left = importdata('side/base_side_pick_left.txt');
data_pick_mid = importdata('side/base_side_pick_mid.txt');
data_pick_right = importdata('side/base_side_pick_right.txt');

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

data_place = importdata('side/base_pos_side_place.txt');
data_pick_left = importdata('side/base_side_pick_left.txt');
data_pick_mid = importdata('side/base_side_pick_mid.txt');
data_pick_right = importdata('side/base_side_pick_right.txt');

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
x = data(:,1);
y = data(:,2);
z = data(:,3);

figure(2)
stem3(x,y,z)
grid on

xv = linspace(min(x), max(x), 20);
yv = linspace(min(y), max(y), 20);
[X,Y] = meshgrid(xv, yv);
Z = griddata(x,y,z,X,Y);
figure(3)
surf(X, Y, Z);
xlim([-0.6 0.5])
ylim([-0.5 0.5])
grid on
set(gca, 'ZLim',[0 100])
shading interp