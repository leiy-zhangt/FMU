%%导入数据
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

R=287.05287;
T=288.15;
L=-0.0065;
P=101325;
G=9.80665;
rho=1.1736;
for i=1:size(alt)
    p_e = 0.5*rho*velocity(i)^2;
    e(i)=T*R*p_e/G*power(pressure(i)/P,L*R/G-1)*(-pressure(i)/P^2);
    height_cali(i)=height(i)+e(i);
end
%%显示高度
figure(1)
plot(height_cali)
hold on
plot(height)
figure(2)
plot(alt)
figure(3)
plot(velocity)
%%气压计与GNSS高度对比
% plot(height_cali-150)
% hold on
% plot(alt)
