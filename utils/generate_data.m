function [model, scene, modelClean, R, t] = generate_data(option, mode)
%GENERATE_DATA generates model and scene data pair such that 
%   scene = model*R' + t' (note the transpose)
%
% INPUT
% option : struct                   - structure with data generation info
%   modelList : cell                - cell of template shapes
%   modelNPTRange : 1 x 2 matrix    - range of number of points in the
%                                     generated shapes
%   angRange : 1 x 2 matrix         - range of rotation (degress)
%   transRange : 1 x 2 matrix       - range of translation
%   noiseSdRange : 1 x 2 matrix     - range of noise sd
%   occlu : 1 x 2 matrix            - range of occlusion ratio
%   outl : 1 x 2 matrix             - range of outlier ratio
%
% OUTPUT
% model : n x 3 matrix          - model shape
% scene : m x 3 matrix          - scene shape
% modelClean : n1 x 3 matrix    - model shape before adding outliers, noise,  
%                                 and occlusion
% R : 3 x 3 matrix              - rotation matrix
% t : 3 x 1 matrix              - translation vector

if ~exist('mode', 'var')
    mode = 'Test';
end

% Get range of points 
nPtRange = option.modelNPtRange;

% Select a model from list
nModel = length(option.modelList);
modelTmp = option.modelList{randRange([1 nModel],1,'int')};
nPtModel = size(modelTmp,1);

% Random points from the model
nModel = randRange(nPtRange,1,'int');
model = modelTmp(randRange([1 nPtModel],[1 nModel],'int'),:);
nScene = randRange(nPtRange,1,'int');
scene = modelTmp(randRange([1 nPtModel],[1 nScene],'int'),:);

% set same shape for training mode
if mode == "training"
    scene = model;
end

% Randomly rotate both pt clouds using same rotation
R = quat2rotm(normc(randn(4,1))');
model = model*R;
scene = scene*R;

% Normalise scale
[model, scene] = normalise(model, scene);

% Save clean model
modelClean = model;

% Create transformations and transform scene shape
R = genRandomRotationAngle(randRange(option.angRange));
t = randRange(option.transRange)*normc(randn(3,1));

% Transform scene model
scene = scene*R'+t';

% Add noise
model = model + randRange(option.noiseSdRange)*randn(size(model));
scene = scene + randRange(option.noiseSdRange)*randn(size(scene));

% Add occlusion on either shape or model
if rand > 0.5
    model = occlude(model,randRange(option.occlu));
else
    scene = occlude(scene,randRange(option.occlu));
end

% Create outliers
outlRad = [-1.25 1.25];
nOutlM = round(size(model,1)*randRange(option.outl));
model = [model; randRange(outlRad,[nOutlM 3])];
nOutlS = round(size(scene,1)*randRange(option.outl));
scene = [scene; randRange(outlRad,[nOutlS 3])];

end

function [ occX ] = occlude(X,perc)
%OCCLUDE occludes a percentage perc of points in X at a random direction
%   X is n x 3

% Calculate number of positions to be occluded
nOfOcc = round(perc*size(X,1));

% Define a random direction
a = normc(randn(1,3));

% Project and sort
proj     = X*a';
[~,inds] = sort(proj,'descend');

% Get desired indices
occX = X(inds(nOfOcc+1:end),:);

end

% normalize shapes
function [Cm, Cs] = normalise(Cm, Cs)

scale = max([Cm(:);Cs(:)]);
Cm = Cm/scale;
Cs = Cs/scale;

end