function [R] = expMapSO3(r)
%EXPMAP : r \in R^3 -> R in SO(3)

theta = norm(r(1:3));

if theta == 0
    R  = eye(3);
else
    wx = [0 -r(3) r(2);r(3) 0 -r(1); -r(2) r(1) 0]/theta;
    sth = sin(theta);
    omcth = 1 - cos(theta);
    wx2 = wx*wx;
    R = eye(3) + sth*wx + omcth*wx2;
end

end

