%%
%自稳飞行数据
figure(1);
index = find(mode>1200)
plot(time(index),pitch(index));
hold on;
plot(time(index),exp_pitch(index));
figure(2);
plot(time(index),roll(index));
hold on;
plot(time(index),exp_roll(index));
%%
%定高模式
index = find(mode>1800)
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
plot(time(index),exp_the(index));
figure(5);
plot(time(index),velocity(index))




