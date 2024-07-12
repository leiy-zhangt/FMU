%%
%保存数据
% clear;
% clc;
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
angle = data.VarName42
mode = data.VarName46;
exp_pitch = data.VarName48;
exp_roll = data.VarName50;
exp_yaw = data.VarName52;
exp_thrust = data.VarName54;
exp_height = data.VarName56;
servo_pitch = data.VarName58;
servo_roll = data.VarName60;
%%
clear;
close all;
clc;
load('data_2024_7_5_8_34_16.mat');
%%
%自稳飞行数据
close all;
figure(1);
index = find((mode>1400));
plot(time(index),pitch(index));
hold on;
plot(time(index),exp_pitch(index));
figure(2);
plot(time(index),roll(index));
hold on;
plot(time(index),exp_roll(index));
figure(3);
plot(time(index),yaw(index));
hold on;
plot(time(index),angle(index));

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
%%
%问题分析
close all;
figure(1);
plot(time,(mode-1000)/10);
hold on;
plot(time,pitch);
hold on;
plot(time,gx);
figure(2);
plot(time,(mode-1000)/10);
hold on;
plot(time,exp_pitch);
hold on;
plot(time,servo_pitch);
figure(3);
out = 3*(exp_pitch-pitch)-0.5*gx;
plot(time,(mode-1000)/10);
%%
%姿态计算程序测试
close all
clear tpitch troll tyaw tq index
tpitch(1) = pitch(1);
troll(1) = roll(1);
tyaw(1) = yaw(1);
tq=zeros(1,4);
% starttime = 10400
starttime = 2000;
[tpitch(1) troll(1) tyaw(1) tq(1,:)] = euler2q([pitch(starttime+1) roll(starttime+1) yaw(starttime+1)],[0 0 0]);
for i=1:9260
     [tpitch(i+1) troll(i+1) tyaw(i+1) tq(i+1,:)] = euler2q([tpitch(i) troll(i) tyaw(i)],[gx(starttime+i) gy(starttime+i) gz(starttime+i)]);
end
figure(1)
plot(tpitch);
hold on
plot(pitch(starttime+1:starttime+max(size(tpitch))));
figure(2)
plot(tpitch'-pitch(starttime+1:starttime+max(size(tpitch))));
figure(3)
plot(troll);
hold on
plot(roll(starttime+1:starttime+max(size(tpitch))));
figure(4)
plot(troll'-roll(starttime+1:starttime+max(size(tpitch))));
figure(5)
plot(tyaw);
hold on
plot(yaw(starttime+1:starttime+max(size(tpitch))));
figure(6)
err = tyaw'-yaw(starttime+1:starttime+max(size(tpitch)));
index = find(err<-180);
err(index) = err(index) + 360;
clear index
index = find(err>180);
err(index) = err(index) - 360;
plot(err);
figure(7)
plot(sqrt(ax(starttime+1:starttime+max(size(tpitch))).^2+ay(starttime+1:starttime+max(size(tpitch))).^2+az(starttime+1:starttime+max(size(tpitch))).^2));


