function [rdiff] = computeRotationDiff(R1, R2)
%CALERRROT Compute the difference between two rotation matrices in radians
    rdiff = 2*asin(norm(R1-R2,'fro')/(2*sqrt(2)));
    
end

