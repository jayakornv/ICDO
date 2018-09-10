function [D] = pwSqDist(X,Y)
%PWSQDIST compute pairwise squared distances  between columns in X and Y
%
% INPUT 
% X : d x n matrix  
% Y : d x m matrix
%
% OUTPUT
% D : n x m matrix - pariwise squared distance

 
X = X';

sX = size(X);
sY = size(Y);

% temporary matrix
tmpX = ones(sX(1),3*sX(2));
tmpY = ones(3*sY(1),sY(2));

tmpX(:,2:3:end) = -2*X;
tmpX(:,3:3:end) = X.*X; % faster than X.^2
tmpY(1:3:end,:) = Y.*Y; % faster than Y.^2
tmpY(2:3:end,:) = Y;

D = tmpX*tmpY;
end

