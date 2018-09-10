function [val] = randRange(range,num,type)    
% RANDRANGE random number within specified range
%
% INPUT
% range : 1 x 2 matrix - range to randomize from
% num : matrix - size to generate
% type: "double" "float" or "int"
%
% OUTPUT
% val : matrix of size num - generated numbers

    if ~exist('num','var')
        num = 1;
    end
    
    if ~exist('type','var')
        type = 'double';
    end
    
    if strcmp(type,'double') || strcmp(type,'float')
        
        % random double
        val = (range(2)-range(1))*rand(num)+range(1);
        
    elseif strcmp(type,'int')
        
        % round the edge
        range(1) = ceil(range(1));
        range(2) = floor(range(2));
        
        % random
        val = floor((range(2)-range(1)+1)*rand(num)+range(1));
    end
        
end

