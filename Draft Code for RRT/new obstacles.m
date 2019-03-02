clear;
clc;
 
k = [2, 10]; %Start point
scatter(k(1), k(2),100,'r','filled')
hold on

side1 = 24;
side2 = 18; %lengths of rectangle grid

lineSize = 0.5; %The length of the line connecting nodes
pointHistory(1,:) = k; %column vector that records all node points

%Obstacles-----------------------------------------------------------------

x_obs = [8, 10, 10, 8, 8]; %obstacle 1
y_obs = [18, 18, 12, 12, 18];
x_obs2 = [12, 14, 14, 12, 12]; %obstacle 2
y_obs2 = [3, 3, 0, 0, 3];
x_obs3 = [22, 24,  24, 22, 22]; %obstacle 3
y_obs3 = [3, 3, 0, 0, 3];
x_obs4 = [15, 17, 17, 15, 15]; %obstacle 4
y_obs4 = [18, 18, 15, 15, 18];
x_obs5 = [22, 24,  24, 22, 22]; %obstacle 5
y_obs5 = [18, 18, 15, 15, 18];
x_obs6 = [0, 3,  3, 0, 0]; %obstacle 6
y_obs6 = [8, 8, 5, 5, 8];

%Obstacles-------------------------------(fake)----------------------------
x_obs_B = [7, 11, 11, 7, 7]; 
y_obs_B = [18, 18, 11, 11, 18];
x_obs2_B = [11, 11, 15, 15, 11];
y_obs2_B = [0, 4, 4, 0, 0];
x_obs3_B = [21, 21, 24, 24, 21];
y_obs3_B = [0, 4, 4, 0, 0];
x_obs4_B = [14, 14, 18, 18, 14]; 
y_obs4_B = [18, 14, 14, 18, 18];
x_obs5_B = [21, 21, 24, 24, 21]; 
y_obs5_B = [18, 14, 14, 18, 18];
x_obs6_B = [0, 4, 4, 0, 0]; 
y_obs6_B = [9, 9, 4, 4, 9];
%Obstacles----------------------------(fake up to here)--------------------
plot( x_obs6, y_obs6, 'k')
fill( x_obs6, y_obs6, 'k')
plot(x_obs, y_obs, 'k')
fill(x_obs, y_obs, 'k')
plot( x_obs2, y_obs2, 'k')
fill( x_obs2, y_obs2, 'k')
plot( x_obs3, y_obs3, 'k')
fill( x_obs3, y_obs3, 'k')
plot( x_obs4, y_obs4, 'k')
fill( x_obs4, y_obs4, 'k')
plot( x_obs5, y_obs5, 'k')
fill( x_obs5, y_obs5, 'k')
hold on

%Creating grid-------------------------------------------------------------
x_grid = [0 0 24 24 0];
y_grid = [0 18 18 0 0];
plot(x_grid,y_grid, 'k')
axis([0 24 0 18])
hold on
grid on
grid minor

%Creating the end goal circle----------------------------------------------
x_center = 18;
y_center = 1.5;
radius = 0.8;

theta = 0:0.01:2*pi;

x_goal = radius *cos(theta) + x_center;
y_goal = radius *sin(theta) + y_center;

fill(x_goal,y_goal,'g');
hold on
