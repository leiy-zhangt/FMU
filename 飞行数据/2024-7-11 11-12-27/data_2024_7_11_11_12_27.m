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
spitch = data.VarName62;
sroll = data.VarName64;
syaw = data.VarName66;
%%
clear;
close all;
clc;
load('data_2024_7_11_11_12_27.mat');
%%
%全部数据绘制
close all;
figure(1);
plot(time,spitch);
hold on;
plot(time,exp_pitch);
hold on;
plot(time,pitch);
legend('spitch','epitch','pitch');
figure(2);
plot(time,sroll);
hold on;
plot(time,exp_roll);
hold on;
plot(time,roll);
legend('sroll','eroll','roll');
figure(3);
plot(time,syaw);
hold on;
plot(time,angle);
hold on;
plot(time,yaw);
legend('syaw','angle','yaw');
figure(4);
plot(time,height);
hold on;
plot(time,exp_height);
hold on;
plot(time,(mode-1000));
%%
%自稳飞行数据
close all;
figure(1);
index = find((mode>1400));
plot(time(index),spitch(index));
hold on;
plot(time(index),exp_pitch(index));
hold on;
plot(time(index),pitch(index));
legend('spitch','epitch','pitch');
figure(2);
plot(time(index),sroll(index));
hold on;
plot(time(index),exp_roll(index));
hold on;
plot(time(index),roll(index));
legend('sroll','eroll','roll');
figure(3);
plot(time(index),syaw(index));
hold on;
plot(time(index),angle(index));
hold on;
plot(time(index),yaw(index));
legend('syaw','angle','yaw');
%%
%定高模式
close all;
index = find(mode>1800);
figure(1);
plot(time(index),sroll(index));
hold on;
plot(time(index),exp_roll(index));
hold on;
plot(time(index),roll(index));
legend('sroll','eroll','roll');
figure(2);
plot(time(index),height(index));
hold on;
plot(time(index),exp_height(index));
figure(3);
plot(time(index),spitch(index));
hold on;
plot(time(index),exp_pitch(index));
hold on;
plot(time(index),pitch(index));
legend('spitch','epitch','pitch');
figure(4);
plot(time(index),exp_thrust(index));
figure(5);
plot(time(index),velocity(index))
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
starttime = 0;
[tpitch(1) troll(1) tyaw(1) tq(1,:)] = euler2q([pitch(starttime+1) roll(starttime+1) yaw(starttime+1)],[0 0 0]);
for i=1:30900
     [tpitch(i+1) troll(i+1) tyaw(i+1) tq(i+1,:)] = euler2q([tpitch(i) troll(i) tyaw(i)],[gx(starttime+i) gy(starttime+i) gz(starttime+i)]);
     if(ax(starttime+i)^2+ay(starttime+i)^2+az(starttime+i)^2<15^2)
%          [tpitch(i+1) troll(i+1) tyaw(i+1)] = 0.99*[tpitch(i+1) troll(i+1) tyaw(i+1)] + (1-0.99)*[pitch(starttime+i+1) roll(starttime+i+1) yaw(starttime+i+1)];
        tpitch(i+1) = 0.99*tpitch(i+1)+0.01*pitch(starttime+i+1);
        troll(i+1) = 0.99*troll(i+1)+0.01*roll(starttime+i+1);
        tyaw(i+1) = 0.99*tyaw(i+1)+0.01*yaw(starttime+i+1);
     end
end
figure(1)
plot(tpitch);
hold on
plot(pitch(starttime+1:starttime+max(size(tpitch))));
hold on
plot(spitch(starttime+1:starttime+max(size(tpitch))));
legend('tpitch','pitch','spitch');
% figure(2)
% plot(tpitch'-pitch(starttime+1:starttime+max(size(tpitch))));
figure(3)
plot(troll);
hold on
plot(roll(starttime+1:starttime+max(size(tpitch))));
hold on
plot(sroll(starttime+1:starttime+max(size(tpitch))));
legend('troll','roll','sroll');
% figure(4)
% plot(troll'-roll(starttime+1:starttime+max(size(tpitch))));
figure(5)
plot(tyaw);
hold on
plot(yaw(starttime+1:starttime+max(size(tpitch))));
hold on
plot(syaw(starttime+1:starttime+max(size(tpitch))));
% figure(6)
% err = tyaw'-yaw(starttime+1:starttime+max(size(tpitch)));
% index = find(err<-180);
% err(index) = err(index) + 360;
% clear index
% index = find(err>180);
% err(index) = err(index) - 360;
% plot(err);
% figure(7)
% plot(sqrt(ax(starttime+1:starttime+max(size(tpitch))).^2+ay(starttime+1:starttime+max(size(tpitch))).^2+az(starttime+1:starttime+max(size(tpitch))).^2));


