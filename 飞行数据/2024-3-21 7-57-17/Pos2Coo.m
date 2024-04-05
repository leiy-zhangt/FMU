function [dpx,dpy] = Pos2Coo(lon1,lat1,lon2,lat2)
%将两个经纬度装换为坐标差
%   将坐标1与坐标2的经纬度转换为坐标差
    dpx = (lon1-lon2)*100000;
    dpy = (lat1-lat2)*111320;
end

