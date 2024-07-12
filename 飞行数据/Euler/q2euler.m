function [pitch,roll,yaw] = q2euler(q)
%Q2 四元数转换为欧拉角
%   输入值为四元数，返回值为俯仰角、滚转角、偏航角
    q_norm = q(1)*q(1)+q(2)*q(2)+q(3)*q(3)+q(4)*q(4);
    q_norm = sqrt(q_norm);
    q(1) = q(1)/q_norm;
    q(2) = q(2)/q_norm;
    q(3) = q(3)/q_norm;
    q(4) = q(4)/q_norm;
    pitch = asin(2*(q(1)*q(2)+q(3)*q(4)))*57.3;
    yaw = atan2(-2*(q(2)*q(3)-q(1)*q(4)),(power(q(1),2)-power(q(2),2)+power(q(3),2)-power(q(4),2)))*57.3;
    roll = atan2(-2*(q(2)*q(4)-q(1)*q(3)),power(q(1),2)-power(q(2),2)-power(q(3),2)+power(q(4),2))*57.3;
    if roll<-90
		roll = 180 + roll;
		if pitch>0  
            pitch=180-pitch;
        else
            pitch=-180-pitch;
        end
        if yaw>0  
            yaw=yaw-180;
        else
            yaw=yaw+180;
        end
    elseif roll>90
        roll = roll -180;
        if pitch>0  
            pitch=180-pitch;
        else
            pitch=-180-pitch;
        end 
        if yaw>0  
            yaw=yaw-180;
        else
            yaw=yaw+180;
        end
    end
end

