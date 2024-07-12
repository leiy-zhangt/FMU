function [pitch roll yaw q] = euler2q(euler,gyr)
%EULER2Q 将角速度转换为欧拉角
%   将当前的角速度转换到姿态角，返回值为俯仰，滚转，偏航
    q = zeros(4,1);
    w = zeros(3,1);
    dq = zeros(4,1);
    pitch = euler(1)/57.3;
    roll = euler(2)/57.3;
    yaw = euler(3)/57.3;
    T_11 = cos(roll)*cos(yaw)-sin(roll)*sin(pitch)*sin(yaw);
    T_21 = cos(roll)*sin(yaw)+sin(roll)*sin(pitch)*cos(yaw);
    T_31 = -sin(roll)*cos(pitch);
    T_12 = -cos(pitch)*sin(yaw);
    T_22 = cos(pitch)*cos(yaw);
    T_32 = sin(pitch);
    T_13 = sin(roll)*cos(yaw)+cos(roll)*sin(pitch)*sin(yaw);
    T_23 = sin(roll)*sin(yaw)-cos(roll)*sin(pitch)*cos(yaw);
    T_33 = cos(roll)*cos(pitch);
    q(1) = 0.5*sqrt(1+T_11+T_22+T_33);
    q(2) = 0.5*sqrt(1+T_11-T_22-T_33);
    q(3) = 0.5*sqrt(1-T_11+T_22-T_33);
    q(4) = 0.5*sqrt(1-T_11-T_22+T_33);
    if((T_32 - T_23)<0) q(2) = -q(2); end
    if((T_13 - T_31)<0) q(3) = -q(3); end
    if((T_21 - T_12)<0) q(4) = -q(4); end
    %更新四元数
    w(1) = gyr(1) /57.3;
    w(2) = gyr(2) /57.3;
    w(3) = gyr(3) /57.3;
    dq(1) = 0.5*(-w(1)*q(2)-w(2)*q(3)-w(3)*q(4));
    dq(2) = 0.5*(w(1)*q(1)+w(3)*q(3)-w(2)*q(4));
    dq(3) = 0.5*(w(2)*q(1)-w(3)*q(2)+w(1)*q(4));
    dq(4) = 0.5*(w(3)*q(1)+w(2)*q(2)-w(1)*q(3));
    dt = 0.005;
    q(1) = q(1) + dq(1)*dt;
    q(2) = q(2) + dq(2)*dt;
    q(3) = q(3) + dq(3)*dt;
    q(4) = q(4) + dq(4)*dt;
    q_norm = q(1)*q(1)+q(2)*q(2)+q(3)*q(3)+q(4)*q(4);
    q_norm = sqrt(q_norm);
    q(1) = q(1)/q_norm;
    q(2) = q(2)/q_norm;
    q(3) = q(3)/q_norm;
    q(4) = q(4)/q_norm;
    %求解矩阵
    [pitch,roll,yaw]= q2euler(q);
end

