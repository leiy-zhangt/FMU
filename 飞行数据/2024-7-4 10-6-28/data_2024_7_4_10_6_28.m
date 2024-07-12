%%
%保存数据
clear;
clc;
time = data.VarName2;
ax = data.VarName6;
ay = data.VarName8;
az = data.VarName10;
gx = data.VarName12;
gy = data.VarName14;
gz = data.VarName16;
pitch = data.VarName18;
roll = data.VarName20;
yaw = data.VarName22;
pressure = data.VarName24;
height = data.VarName26;
lon = data.VarName30;
lat = data.VarName32;
alt = data.VarName34;
velocity = data.VarName36;
ve = data.VarName38;
vn = data.VarName40;
exp_height = data.VarName42;
mode = data.VarName46;
exp_pitch = data.VarName48;
exp_roll = data.VarName50;
exp_yaw = data.VarName52;
exp_thrust = data.VarName54;
servo_pitch = data.VarName56;
servo_roll = data.VarName58;
%%
clear;
close all;
clc;
load('data_2024_7_4_10_6_28.mat');
%%
%自稳飞行数据
close all;
figure(1);
index = find((mode>1400) & (mode<1600));
plot(time(index),pitch(index));
hold on;
plot(time(index),exp_pitch(index));
figure(2);
plot(time(index),roll(index));
hold on;
plot(time(index),exp_roll(index));
%%
%定高模式
close all;
index = find(mode>1800);
figure(1);
plot(time(index),roll(index));
hold on;
plot(time(index),exp_roll(index));
figure(2);
plot(time(index),height(index));
hold on;
plot(time(index),exp_height(index));
figure(3);
plot(time(index),pitch(index));
hold on;
plot(time(index),exp_pitch(index));
figure(4);
plot(time(index),exp_thrust(index));
figure(5);
plot(time(index),velocity(index))
figure(6);
plot(time,pitch);
