function [R] = genRandomRotationAngle(angle)
%GENRANDOMROTATIONANGLE generate a random rotation matrix at the specified
%angle

k = randn(3,1);
k = k/sqrt(k'*k);
K = [0 -k(3) k(2);k(3) 0 -k(1);-k(2) k(1) 0];
R = eye(3)+sin(angle/180*pi)*K+(1-cos(angle/180*pi))*K^2;
end

