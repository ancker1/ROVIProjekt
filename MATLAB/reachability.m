%% Figure 1
clc; clear; close all;

data_pick = importdata('base_side_pick_1000.txt');
data_place = importdata('base_side_place_1000.txt');

data = importdata('base_side_pick_1000.txt');
data(:,3) = data_pick(:,3) + data_place(:,3);

tablex = [-40 40 40 -40];
tabley = [-60 -60 60 60];

pickx = [-40 40 40 -40];
picky = [-60 -60 -30 -30];

placex = [-40 -20 -20 -40];
placey = [40 40 60 60];

colormap default
cmap = parula(1+max(data(:,3)));

%figure
figure(1)
fill(tablex,tabley,[0.9 0.9 0.9])
hold on
fill(pickx, picky, [0.9 0.9 0])
fill(placex, placey, [0 0.8 0])
sz = 150;

c = cmap(data(:,3)+1,:);

scatter(-100*data(:,1), -100*data(:,2),sz,c,'filled')
scatter(-100*[-0.25],-100*[0.474],400,[0.50 0.50 0.50], 'filled')
colorbar

text(-12, -45, 'Pick area');
text(-38,55,'Place')
text(-37,48,'area')

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