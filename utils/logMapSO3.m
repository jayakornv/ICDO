function [l] = logMapSO3(R)
%LOGMAP : R in SO(3) -> r \in R^3

v = (trace(R)-1)/2;
v = min(1,max(-1,v));

if abs(v+1) < 1e-10
    warning('180 degrees rotation: representation not unique');
end

if abs(v-1) < 1e-12
    l = zeros(3,1);
else
    theta = acos(v);
    wx = theta/(2*sin(theta))*(R-R');
    l = [-wx(2,3); wx(1,3); -wx(1,2)];
end

end


