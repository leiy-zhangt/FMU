%%导入数据
clear;
clc;
load('data_2024_3_21_8_11_54.mat');
time = data_2024_3_21_8_11_54.VarName2;
ax = data_2024_3_21_8_11_54.VarName6;
ay = data_2024_3_21_8_11_54.VarName8;
az = data_2024_3_21_8_11_54.VarName10;
gx = data_2024_3_21_8_11_54.VarName12;
gy = data_2024_3_21_8_11_54.VarName14;
gz = data_2024_3_21_8_11_54.VarName16;
pitch = data_2024_3_21_8_11_54.VarName18;
roll = data_2024_3_21_8_11_54.VarName20;
yaw = data_2024_3_21_8_11_54.VarName22;
pressure = data_2024_3_21_8_11_54.VarName24;
height = data_2024_3_21_8_11_54.VarName26;
lon = data_2024_3_21_8_11_54.VarName30;
lat = data_2024_3_21_8_11_54.VarName32;
alt = data_2024_3_21_8_11_54.VarName34;
v = data_2024_3_21_8_11_54.VarName36;
ve = data_2024_3_21_8_11_54.VarName38;
vn = data_2024_3_21_8_11_54.VarName40;
height_expect = data_2024_3_21_8_11_54.VarName42;
channel_1 = data_2024_3_21_8_11_54.VarName46;
channel_2 = data_2024_3_21_8_11_54.VarName48;
channel_3 = data_2024_3_21_8_11_54.VarName50;
channel_4 = data_2024_3_21_8_11_54.VarName52;
channel_5 = data_2024_3_21_8_11_54.VarName54;
channel_6 = data_2024_3_21_8_11_54.VarName56;
channel_7 = data_2024_3_21_8_11_54.VarName58;
channel_8 = data_2024_3_21_8_11_54.VarName60;

% R=287.05287;
% T=288.15;
% L=-0.0065;
% P=101325;
% G=9.80665;
% rho=1.1736;
% for i=1:size(alt)
%     p_e = 0.5*rho*velocity(i)^2;
%     e(i)=T*R*p_e/G*power(pressure(i)/P,L*R/G-1)*(-pressure(i)/P^2);
%     height_cali(i)=height(i)+e(i);
% end
% %%显示高度
% figure(1)
% plot(height_cali)
% hold on
% plot(height)
% figure(2)
% plot(alt)
% figure(3)
% plot(velocity)
%%气压计与GNSS高度对比
% plot(height_cali-150)
% hold on
% plot(alt)
%%
% n=1;
% for i=1:size(alt)
%     if channel_6(i) > 1400 & channel_6(i) < 1600
%         pitch_expect(n) = (channel_2(i)-1500)*0.12;
%         pitch_true(n) = pitch(i);
%         n = n+1;
%     end
% end
% plot(pitch_true);
% hold on;
% plot(pitch_expect);
%%
% n=1;
% for i=1:size(alt)
%     if channel_6(i) > 1400 & channel_6(i) < 1600
%         roll_expect(n) = (channel_1(i)-1500)*0.12;
%         roll_true(n) = roll(i);
%         n = n+1;
%     end
% end
% plot(roll_true);
% hold on;
% plot(roll_expect);
%%
n=1;
for i=1:size(alt)
    if channel_6(i) > 1600
        roll_expect(n) = (channel_1(i)-1500)*0.12;
        roll_true(n) = roll(i);
        pitch_expect(n) = 3*(height_expect(i)-alt(i));
        pitch_true(n) = pitch(i);
        hei_exp(n)=height_expect(i);
        height_true(n)=alt(i);
        height_pre(n) = height(i);
        n = n+1;
    end
end
figure(1);
plot(roll_true);
hold on;
plot(roll_expect);
figure(2);
plot(pitch_true);
hold on;
plot(pitch_expect);
figure(3);
plot(hei_exp);
hold on;
plot(height_true);
figure(4);
plot(height_pre);

%%
%分析加速度噪声
[b,a] = butter(4,1/50);
ax_fil = filter(b,a,ax);
v(999)=0;
% for i=2:numel(ax)
for i=1000:1500
    % v(i)=v(i-1)+ax_fil(i);
    v(i)=v(i-1)+ay(i)*0.01;
end
plot(time(1000:1500),v(1000:1500))
hold on 
plot(time(1000:1500),ax(1000:1500))