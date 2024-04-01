%%导入数据
clear;
clc;
load('data_2024_3_21_7_57_17.mat');
time = data_2024_3_21_7_57_17.VarName2;
ax = data_2024_3_21_7_57_17.VarName6;
ay = data_2024_3_21_7_57_17.VarName8;
az = data_2024_3_21_7_57_17.VarName10;
gx = data_2024_3_21_7_57_17.VarName12;
gy = data_2024_3_21_7_57_17.VarName14;
gz = data_2024_3_21_7_57_17.VarName16;
pitch = data_2024_3_21_7_57_17.VarName18;
roll = data_2024_3_21_7_57_17.VarName20;
yaw = data_2024_3_21_7_57_17.VarName22;
pressure = data_2024_3_21_7_57_17.VarName24;
height = data_2024_3_21_7_57_17.VarName26;
lon = data_2024_3_21_7_57_17.VarName30;
lat = data_2024_3_21_7_57_17.VarName32;
alt = data_2024_3_21_7_57_17.VarName34;
v = data_2024_3_21_7_57_17.VarName36;
ve = data_2024_3_21_7_57_17.VarName38;
vn = data_2024_3_21_7_57_17.VarName40;
height_expect = data_2024_3_21_7_57_17.VarName42;
channel_1 = data_2024_3_21_7_57_17.VarName46;
channel_2 = data_2024_3_21_7_57_17.VarName48;
channel_3 = data_2024_3_21_7_57_17.VarName50;
channel_4 = data_2024_3_21_7_57_17.VarName52;
channel_5 = data_2024_3_21_7_57_17.VarName54;
channel_6 = data_2024_3_21_7_57_17.VarName56;
channel_7 = data_2024_3_21_7_57_17.VarName58;
channel_8 = data_2024_3_21_7_57_17.VarName60;

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
%对气压计高度进行补偿
% n=1;
% for i=1:size(alt)
%     if channel_6(i) > 1600
%         roll_expect(n) = (channel_1(i)-1500)*0.12;
%         roll_true(n) = roll(i);
%         pitch_expect(n) = 3*(height_expect(i)-alt(i));
%         pitch_true(n) = pitch(i);
%         hei_exp(n)=height_expect(i);
%         height_true(n)=alt(i);
%         height_pre(n) = height(i);
%         n = n+1;
%     end
% end
% figure(1);
% plot(roll_true);
% hold on;
% plot(roll_expect);
% figure(2);
% plot(pitch_true);
% hold on;
% plot(pitch_expect);
% figure(3);
% plot(hei_exp);
% hold on;
% plot(height_true);
% figure(4);
% plot(height_pre);


%%
% 对加速度进行FFT
% [len,i] = size(ax);
% if mod(len,2) == 1
%     len = len-1;
% end
% yax = fft(ax(1:len)); 
% %将横坐标转化，显示为频率f= n*(fs/N)
% f = (0:len/2-1)*100/len;
% realyax=2*abs(yax(1:len/2))/len;
% figure(1);
% plot(f,realyax);
% title('ax');
% 
% [len,i] = size(ay);
% if mod(len,2) == 1
%     len = len-1;
% end
% yay = fft(ay(1:len)); 
% %将横坐标转化，显示为频率f= n*(fs/N)
% f = (0:len/2-1)*100/len;
% realyay=2*abs(yay(1:len/2))/len;
% figure(2);
% plot(f,realyay);
% title('ay');
% 
% 
% [len,i] = size(az);
% if mod(len,2) == 1
%     len = len-1;
% end
% yaz = fft(az(1:len)); 
% %将横坐标转化，显示为频率f= n*(fs/N)
% f = (0:len/2-1)*100/len;
% realyaz=2*abs(yaz(1:len/2))/len;
% figure(3);
% plot(f,realyaz);
% title('az');

%%
% 对数据进行滤波处理
[b,a] = butter(4,1/50);
ax_fil = filter(b,a,ax);
figure(1);
plot(time,ax_fil);
[b,a] = butter(4,1/50);
ay_fil = filter(b,a,ay);
figure(2);
plot(time,ay_fil);
[b,a] = butter(4,1/50);
az_fil = filter(b,a,az);
figure(3);
plot(time,az_fil);


