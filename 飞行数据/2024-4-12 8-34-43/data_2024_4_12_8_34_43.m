%%
%飞行数据处理
clear;
clc;
load('data_2024_4_12_8_34_43.mat');
time = data_2024_4_12_8_34_43.VarName2;
ax = data_2024_4_12_8_34_43.VarName6;
ay = data_2024_4_12_8_34_43.VarName8;
az = data_2024_4_12_8_34_43.VarName10;
gx = data_2024_4_12_8_34_43.VarName12;
gy = data_2024_4_12_8_34_43.VarName14;
gz = data_2024_4_12_8_34_43.VarName16;
pitch = data_2024_4_12_8_34_43.VarName18;
roll = data_2024_4_12_8_34_43.VarName20;
yaw = data_2024_4_12_8_34_43.VarName22;
pressure = data_2024_4_12_8_34_43.VarName24;
height = data_2024_4_12_8_34_43.VarName26;
lon = data_2024_4_12_8_34_43.VarName30;
lat = data_2024_4_12_8_34_43.VarName32;
alt = data_2024_4_12_8_34_43.VarName34;
v = data_2024_4_12_8_34_43.VarName36;
ve = data_2024_4_12_8_34_43.VarName38;
vn = data_2024_4_12_8_34_43.VarName40;
height_expect = data_2024_4_12_8_34_43.VarName42;
channel_1 = data_2024_4_12_8_34_43.VarName46;
channel_2 = data_2024_4_12_8_34_43.VarName48;
channel_3 = data_2024_4_12_8_34_43.VarName50;
channel_4 = data_2024_4_12_8_34_43.VarName52;
channel_5 = data_2024_4_12_8_34_43.VarName54;
channel_6 = data_2024_4_12_8_34_43.VarName56;
channel_7 = data_2024_4_12_8_34_43.VarName58;
channel_8 = data_2024_4_12_8_34_43.VarName60;

%%
%气压计数据
R=287.05287;
T=288.15;
L=-0.0065;
P=101325;
G=9.80665;
rho=1.1736;
for i=1:size(alt)
    p_e = 0.5*rho*v(i)^2;
    e(i)=T*R*p_e/G*power(pressure(i)/P,L*R/G-1)*(-pressure(i)/P^2);
    height_cali(i)=height(i)+e(i);
end
%对比高度数据
figure(1)
plot(height_cali);
hold on
plot(height);
figure(2)
plot(alt);
figure(3)
plot(height_cali)
hold on
plot(alt)
%%
%绘制三维曲线
lat0=lat(1);
lon0=lon(1);
for i=1:numel(ax)
    [x(i),y(i)]=Pos2Coo(lon(i),lat(i),lon0,lat0);
end
plot3(x,y,alt);
xlabel('东向距离/m');
ylabel('北向距离/m');
zlabel('东向距离/m');
title('无人机航迹');
%%
%自稳模式姿态数据
n=1;
for i=1:size(alt)
    if channel_6(i) > 1400 & channel_6(i) < 1600
        pitch_expect(n) = (channel_2(i)-1350)*0.12;
        pitch_true(n) = pitch(i);
        n = n+1;
    end
end
figure(1);
plot(pitch_true);
hold on;
plot(pitch_expect);
%
n=1;
for i=1:size(alt)
    if channel_6(i) > 1400 & channel_6(i) < 1600
        roll_expect(n) = (channel_1(i)-1532)*0.12;
        roll_true(n) = roll(i);
        n = n+1;
    end
end
figure(2)
plot(roll_true);
hold on;
plot(roll_expect);
%%
%登高飞行绘制飞机的高度与姿态曲线
n=1;
for i=1:size(alt)
    if channel_6(i) > 1600
        roll_expect(n) = (channel_1(i)-1532)*0.12;
        roll_true(n) = roll(i);
        pitch_expect(n) = 2*(height_expect(i)-alt(i))+6;
        pitch_true(n) = pitch(i);
        hei_exp(n)=height_expect(i);
        height_true(n)=alt(i);
        height_pre(n) = height(i);
        v_true(n) = v(i);
        power(n) = (channel_3(i)-1000)/1000*20;
        n = n+1;
    end
end
figure(1);
plot(roll_true);
hold on;
plot(roll_expect);
legend({'roll','roll-exp'});
xlabel('时间/s');
ylabel('滚转角/°');
title('无人机滚转角');
figure(2);
plot(pitch_true);
hold on;
plot(pitch_expect);
legend({'pitch','pitch-exp'});
xlabel('时间/s');
ylabel('俯仰角/°');
title('无人机俯仰角');
figure(3);
plot(hei_exp);
hold on;
plot(height_true);
legend({'hight-exp','height'});
xlabel('时间/s');
ylabel('高度/m');
title('无人机高度');
figure(4);
plot(height_pre);
hold on 
plot(hei_exp);
legend({'hight-exp','height'});
xlabel('时间/s');
ylabel('高度/m');
title('无人机高度');
figure(5);
plot(v_true);
hold on;
plot(height_true-hei_exp);
hold on;
plot(power);
legend({'v','height','power'});
xlabel('时间/s');
ylabel('速度/m/s');
title('无人机速度');

%%
%对数据进行FFT变换
[len,i] = size(ax);
if mod(len,2) == 1
    len = len-1;
end
yax = fft(ax(1:len)); 
f = (0:len/2-1)*100/len;
realyax=2*abs(yax(1:len/2))/len;
figure(1);
plot(f,realyax);
title('ax');

[len,i] = size(ay);
if mod(len,2) == 1
    len = len-1;
end
yay = fft(ay(1:len)); 
%%
%FFT的更坐标轴f= n*(fs/N)
f = (0:len/2-1)*100/len;
realyay=2*abs(yay(1:len/2))/len;
figure(2);
plot(f,realyay);
title('ay');

[len,i] = size(az);
if mod(len,2) == 1
    len = len-1;
end
yaz = fft(az(1:len)); 
f = (0:len/2-1)*100/len;
realyaz=2*abs(yaz(1:len/2))/len;
figure(3);
plot(f,realyaz);
title('az');

%%
%对数据进行滤波处理
[b,a] = butter(4,1/50);
ax_fil = filter(b,a,ax);
% figure(1);
% plot(time,ax_fil);
[b,a] = butter(4,1/50);
ay_fil = filter(b,a,ay);
% figure(2);
% plot(time,ay_fil);
[b,a] = butter(4,1/50);
az_fil = filter(b,a,az);
% figure(3);
% plot(time,az_fil);
figure(1);
% plot(time,ax_fil);
% hold on;
% plot(time,ay_fil);
% hold on;
% plot(time,az_fil);
% hold on;
plot(time,ax);
hold on;
plot(time,ay);
hold on;
plot(time,az);
hold on;
legend({'ax','ay','az'});
xlabel('时间/s');
ylabel('加速度/m/s^2');
title('无人机加速度');
[b,a] = butter(4,1/50);
gx_fil = filter(b,a,gx);
[b,a] = butter(4,1/50);
gy_fil = filter(b,a,gy);
[b,a] = butter(4,1/50);
gz_fil = filter(b,a,gz);
figure(2);
plot(time,gx_fil);
hold on;
plot(time,gy_fil);
hold on;
plot(time,gz_fil);
hold on;
legend({'gx','gy','gz'});
xlabel('时间/s');
ylabel('角速度/°/s');
title('无人机角速度');

%%
%对数据进行卡尔曼滤波
k = 0.6;
n_data = cell(6,1);
dt = 0.01;
for i=1:numel(ax)
    if i == 1
        n_data{1,1}(i) = 0;
        n_data{2,1}(i) = 0;
        n_data{3,1}(i) = 0;
        n_data{4,1}(i) = 0;
        n_data{5,1}(i) = 0;
        n_data{6,1}(i) = 0;
    end
    if i ~= 1 
        if lon(i)~=lon(i-1) || lat(i)~=lat(i-1) || alt(i)~=alt(i-1)
            [Px_obe,Py_obe] = Pos2Coo(lon(i),lat(i),lon(1),lat(1));
            n_data{1,1}(i) = k*n_data{1,1}(i) + (1-k)*Px_obe;
            n_data{2,1}(i) = k*n_data{2,1}(i) + (1-k)*Py_obe;
            n_data{3,1}(i) = k*n_data{3,1}(i) + (1-k)*(alt(i)-alt(1));
            n_data{4,1}(i) = k*n_data{4,1}(i) + (1-k)*ve(i);
            n_data{5,1}(i) = k*n_data{5,1}(i) + (1-k)*vn(2);
        end
        n_data{6,1}(i) = k*n_data{6,1}(i) + (1-k)*(height(i)-height(i))/dt;
    end
    T_11 = cos(roll(i)/57.3)*cos(yaw(i)/57.3)-sin(roll(i)/57.3)*sin(pitch(i)/57.3)*sin(yaw(i)/57.3);
	T_21 = cos(roll(i)/57.3)*sin(yaw(i)/57.3)+sin(roll(i)/57.3)*sin(pitch(i)/57.3)*cos(yaw(i)/57.3);
	T_31 = -sin(roll(i)/57.3)*cos(pitch(i)/57.3);
	T_12 = -cos(pitch(i)/57.3)*sin(yaw(i)/57.3);
	T_22 = cos(pitch(i)/57.3)*cos(yaw(i)/57.3);
	T_32 = sin(pitch(i)/57.3);
	T_13 = sin(roll(i)/57.3)*cos(yaw(i)/57.3)+cos(roll(i)/57.3)*sin(pitch(i)/57.3)*sin(yaw(i)/57.3);
	T_23 = sin(roll(i)/57.3)*sin(yaw(i)/57.3)-cos(roll(i)/57.3)*sin(pitch(i)/57.3)*cos(yaw(i)/57.3);
	T_33 = cos(roll(i)/57.3)*cos(pitch(i)/57.3);
	a_e = T_11*ax(i)+T_12*ay(i)+T_13*az(i);
	a_n = T_21*ax(i)+T_22*ay(i)+T_23*az(i);
	a_u = T_31*ax(i)+T_32*ay(i)+T_33*az(i) - 9.80665;
    n_data{1,1}(i+1) = n_data{1,1}(i) + n_data{4,1}(i)*dt + a_e*dt^2/2;
    n_data{2,1}(i+1) = n_data{2,1}(i) + n_data{5,1}(i)*dt + a_n*dt^2/2;
    n_data{3,1}(i+1) = n_data{3,1}(i) + n_data{6,1}(i)*dt + a_u*dt^2/2;
    n_data{4,1}(i+1) = n_data{4,1}(i) + a_e*dt;
    n_data{5,1}(i+1) = n_data{5,1}(i) + a_n*dt;
    n_data{6,1}(i+1) = n_data{6,1}(i) + a_u*dt;
    if height(i)==height(i+1)
        n_data{6,1}(i+1) == 0;
    end
end
figure(1);
plot(time,n_data{1,1}(1:numel(ax)));
hold on;
plot(time,alt);
plot(time,(lon-lon(1))*100000);
figure(2);
plot(time,n_data{2,1}(1:numel(ax)));
hold on;
plot(time,alt);
plot(time,(lat-lat(1))*111320);
figure(3)
plot(time,n_data{3,1}(1:numel(ax)));
hold on;
plot(time,alt);
plot(time,alt-alt(1));
figure(4)
plot(time,n_data{4,1}(1:numel(ax)));
hold on;
plot(time,ve);
figure(5)
plot(time,n_data{5,1}(1:numel(ax)));
hold on;
plot(time,vn);
figure(6)
plot(time,n_data{6,1}(1:numel(ax)));
hold on;
vu(1) = 0;
for i=2:numel(ax)
    vu(i) = (alt(i) - alt(i-1));
end
plot(time,vu);



