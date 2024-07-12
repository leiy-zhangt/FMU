%%
%姿态计算程序测试
% clear;
% clc;
%%
%导入数据
close all
clear tpitch troll tyaw tq index
ax = data.VarName1;
ay = data.VarName2;
az = data.VarName3;
gx = data.VarName4;
gy = data.VarName5;
gz = data.VarName6;
pitch = data.VarName7;
roll = data.VarName8;
yaw = data.VarName9;
atti_pitch = data.VarName10;
atti_roll = data.VarName11;
atti_yaw = data.VarName12;
%%
%原始数据对比
close all;
figure(1)
plot(pitch);
hold on
plot(atti_pitch);
legend('potch','atti_pitch');
figure(2)
plot(pitch-atti_pitch);
figure(3)
plot(roll);
hold on
plot(atti_roll);
legend('roll','atti_roll');
figure(4)
plot(roll-atti_roll);
figure(5)
plot(yaw);
hold on
plot(atti_yaw);
legend('yaw','atti_yaw');
figure(6)
plot(yaw-atti_yaw);
%%
% starttime = 10400
close all;
starttime = 0;
tq=zeros(1,4);
[tpitch(1) troll(1) tyaw(1) tq(1,:)] = euler2q([atti_pitch(starttime+1) atti_roll(starttime+1) atti_yaw(starttime+1)],[0 0 0]);
for i=1:1000
     [tpitch(i+1) troll(i+1) tyaw(i+1) tq(i+1,:)] = euler2q([tpitch(i) troll(i) tyaw(i)],[gx(starttime+i) gy(starttime+i) gz(starttime+i)]);
end
figure(1)
plot(tpitch);
hold on
plot(pitch(starttime+1:starttime+max(size(tpitch))));
hold on
plot(atti_pitch(starttime+1:starttime+max(size(tpitch))));
legend('tp','p','ap');
figure(2)
plot(tpitch'-pitch(starttime+1:starttime+max(size(tpitch))));
figure(3)
plot(troll);
hold on
plot(roll(starttime+1:starttime+max(size(tpitch))));
hold on
plot(atti_roll(starttime+1:starttime+max(size(tpitch))));
legend('tr','r','ar');
figure(4)
plot(troll'-roll(starttime+1:starttime+max(size(tpitch))));
figure(5)
plot(tyaw);
hold on
plot(yaw(starttime+1:starttime+max(size(tpitch))));
hold on
plot(atti_yaw(starttime+1:starttime+max(size(tpitch))));
legend('ty','y','ay');
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