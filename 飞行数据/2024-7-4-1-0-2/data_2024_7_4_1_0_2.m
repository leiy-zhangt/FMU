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
load('data_2024_7_4_1_0_2.mat');
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


% void AttitudeSolution(double gyr_x,double gyr_y,double gyr_z)  //对角速度进行处理，得到弧度值形式的姿态角
% {
%   double w(3),dq(4),q_norm;
%   w(0) = gyr_x * PI /180;
%   w(1) = gyr_y * PI /180;
%   w(2) = gyr_z * PI /180;
%   dq(0) = 0.5*(-w(0)*q(1)-w(1)*q(2)-w(2)*q(3));
%   dq(1) = 0.5*(w(0)*q(0)+w(2)*q(2)-w(1)*q(3));
%   dq(2) = 0.5*(w(1)*q(0)-w(2)*q(1)+w(0)*q(3));
%   dq(3) = 0.5*(w(2)*q(0)+w(1)*q(1)-w(0)*q(2));
%   q(0) = q(0) + dq(0)*dt;
%   q(1) = q(1) + dq(1)*dt;
%   q(2) = q(2) + dq(2)*dt;
%   q(3) = q(3) + dq(3)*dt;
%   q_norm = q(0)*q(0)+q(1)*q(1)+q(2)*q(2)+q(3)*q(3);
%   q_norm = sqrt(q_norm);
%   q(0) = q(0)/q_norm;
%   q(1) = q(1)/q_norm;
%   q(2) = q(2)/q_norm;
%   q(3) = q(3)/q_norm;
%   MotionData.pitch = asin(2*(q(0)*q(1)+q(2)*q(3)));//初始矩阵俯仰角
%   MotionData.yaw = atan2(-2*(q(1)*q(2)-q(0)*q(3)),(pow(q(0),2)-pow(q(1),2)+pow(q(2),2)-pow(q(3),2)));//初始矩阵偏航角
%   MotionData.roll = atan2(-2*(q(1)*q(3)-q(0)*q(2)),pow(q(0),2)-pow(q(1),2)-pow(q(2),2)+pow(q(3),2));//初始矩阵滚转角
%   T_11 = cos(MotionData.roll)*cos(MotionData.yaw)-sin(MotionData.roll)*sin(MotionData.pitch)*sin(MotionData.yaw);
%   T_21 = cos(MotionData.roll)*sin(MotionData.yaw)+sin(MotionData.roll)*sin(MotionData.pitch)*cos(MotionData.yaw);
%   T_31 = -sin(MotionData.roll)*cos(MotionData.pitch);
%   T_12 = -cos(MotionData.pitch)*sin(MotionData.yaw);
%   T_22 = cos(MotionData.pitch)*cos(MotionData.yaw);
%   T_32 = sin(MotionData.pitch);
%   T_13 = sin(MotionData.roll)*cos(MotionData.yaw)+cos(MotionData.roll)*sin(MotionData.pitch)*sin(MotionData.yaw);
%   T_23 = sin(MotionData.roll)*sin(MotionData.yaw)-cos(MotionData.roll)*sin(MotionData.pitch)*cos(MotionData.yaw);
%   T_33 = cos(MotionData.roll)*cos(MotionData.pitch);
% }

%%
%姿态计算程序测试
close all
clear tpitch troll tyaw tq t2pitch t2roll t2yaw t2q
tq=zeros(1,4);
t2q=zeros(1,4);
starttime = 10400;
starttime = 5000;
k = 0.99;%直接结算权重因子
[tpitch(1) troll(1) tyaw(1) tq(1,:)] = euler2q([pitch(starttime+1) roll(starttime+1) yaw(starttime+1)],[0 0 0]);
[t2pitch(1) t2roll(1) t2yaw(1) t2q(1,:)] = euler2q([pitch(starttime+1) roll(starttime+1) yaw(starttime+1)],[0 0 0]);
for i=1:1000
     [tpitch(i+1) troll(i+1) tyaw(i+1) tq(i+1,:)] = euler2q([tpitch(i) troll(i) tyaw(i)],[gx(starttime+i) gy(starttime+i) gz(starttime+i)]);
     [t2pitch(i+1) t2roll(i+1) t2yaw(i+1) t2q(i+1,:)] = euler2q([t2pitch(i) t2roll(i) t2yaw(i)],[gx(starttime+i) gy(starttime+i) gz(starttime+i)]);
     if sqrt(ax(starttime+i)^2+ay(starttime+i)^2+az(starttime+i)^2) < 15
          ans = k*[t2pitch(i+1) t2roll(i+1) t2yaw(i+1)]+(1-k)*[pitch(starttime+i+1) roll(starttime+i+1) yaw(starttime+i+1)];
          t2pitch(i+1) = ans(1); 
          t2roll(i+1) = ans(2); 
          t2yaw(i+1) = ans(3); 
     end
end
figure(1)
plot(tpitch);
hold on
plot(t2pitch);
hold on
plot(pitch(starttime+1:starttime+max(size(tpitch))));
legend('tpitch','t2pitch','pitch');
figure(2)
plot(tpitch'-pitch(starttime+1:starttime+max(size(tpitch))));
hold on
plot(t2pitch'-pitch(starttime+1:starttime+max(size(tpitch))));
legend('tpitch','t2pitch');
figure(3)
plot(troll);
hold on
plot(t2roll);
hold on
plot(roll(starttime+1:starttime+max(size(tpitch))));
legend('troll','t2roll','roll');
figure(4)
plot(troll'-roll(starttime+1:starttime+max(size(tpitch))));
hold on
plot(t2roll'-roll(starttime+1:starttime+max(size(tpitch))));
legend('troll','t2roll');
% figure(5)
% plot(tyaw);
% hold on
% plot(t2yaw);
% hold on
% plot(yaw(starttime+1:starttime+max(size(tpitch))));
% legend('tyaw','t2yaw','yaw');
% figure(6)
% plot(tyaw'-yaw(starttime+1:starttime+max(size(tpitch))));
% hold on
% plot(t2yaw'-yaw(starttime+1:starttime+max(size(tpitch))));
% legend('tyaw','t2yaw');
% figure(7)
% plot(sqrt(ax(starttime+1:starttime+max(size(tpitch))).^2+ay(starttime+1:starttime+max(size(tpitch))).^2+az(starttime+1:starttime+max(size(tpitch))).^2));
%%


%%


