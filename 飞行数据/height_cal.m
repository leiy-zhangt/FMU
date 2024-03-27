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
