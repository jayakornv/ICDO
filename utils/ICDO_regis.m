function [Rout, Tout, Xout, Xstep] = ICDO_regis(model, scene, ICDOmap, maxIter, tol)
%ICDO_EGIS performs registration using ICDO
%
% INPUT
% model : n x 3 matrix      - model shape
% scene : m x 3 matrix      - scene shape
% ICDOmap : struct          - ICDO structure
% maxIter : 1 x 1 matrix    - maximum number of iterations
% tol : 1 x 1 matrix        - tolerance for termination (difference between
%                             current angle and angle in the previous 5
%                             iterations in degress)
%
% OUTPUT
% Rout : 3 x 3 matrix       - result rotation matrix
% Tout : 1 x 3 matrix       - result translation vector
% Xout : 6 x 1 matrix       - result transformation as a 6D vector
% Xstep : 6 x iter          - transformation in each iteration as 6D
%                             vectors
%
% Note: The returned results are parameters that transformed model to
% scene, i.e.,
% scene = model*Rout' + Tout'
%
% In the paper, on the other hand, we describe the approach as estimating
% parameters the transforms Scene to Model. The code here actually follows
% the method in the paper, but only the resulting parameters are inversed
% into those that transform Model to Scene. Also, this code use different
% convention from the paper (column-as-a-3D-point vs row-as-a-3D-point,
% since we try to follow MATLAB pointcloud functions convention here) so
% it might cause some headache if you try to map the code to the method
% in the paper. Sorry for that.
%

% get the maps
Dmap = ICDOmap.D;


% normalize shapes by removing means and scaling
meanModel = sum(model)/size(model,1);
model = model - meanModel;
scene = scene - meanModel;

scale = calScaleFactor(model);
model = model/scale;
scene = scene/scale;

% Parameter initialisation
itStep      = 0;

RtRec = cell(maxIter, 1); % for recording rotation
tRec = zeros(3, maxIter); % for recording translation

diffRot = inf;      % (initial) rotation difference
diffTrans = inf;    % (initial) translation difference
Xstep = nan(6, maxIter);         % for recording transformation in each step
updateStep = nan(6, maxIter);    % for recording update in each step

% ICDO parameters
radius = ICDOmap.radius;
nBox = ICDOmap.nBox;
alpha = ICDOmap.alpha;

% initial transformation
% T contains the transformation where
% model ~ scene*T(1:3,1:3)' + T(1:3, 4)'
T = eye(4);

while itStep < maxIter && (itStep == 1 || diffRot > tol || diffTrans > 3e-3)
    
    itStep = itStep + 1;
    
    % extract feature
    h = compute_h_single(model,scene*T(1:3,1:3)'+T(1:3,4)',radius,nBox, min(itStep,size(Dmap,3)), alpha);
    
    % if iter >= number of maps then use last map
    if itStep <= size(Dmap,3)
        update = Dmap(:,:,itStep)*h;
    else
        update = Dmap(:,:,end)*h;
    end
    
    % average the step to reduce bouncing effect
    if itStep > size(Dmap,3)
        update = (update+updateStep(:,itStep-1))/2;
    end
    
    % save update
    updateStep(:,itStep) = update;
    
    % Convert update into rotation matrix and translation vector.
    % These are transformation such that
    Rupd = expMapSO3(update(1:3));
    tupd = update(4:6);
    
    % update the transformation parameters
    T = [Rupd' -Rupd'*tupd;0 0 0 1]*T;
    
    % save parameters for computing termination criteria
    RtRec{itStep} = T(1:3,1:3)';
    tRec(:,itStep) = T(1:3,4);
    
    % compute termination criteria
    if itStep > 5
        diffRot = computeRotationDiff(RtRec{itStep-5}, RtRec{itStep})/pi*180;
        diffTrans = sqrt(sum((tRec(:,itStep) - tRec(:,itStep-5)).^2));
    else
        diffRot = inf;
        diffTrans = inf;
    end
    
    % save transformation parameters
    Xstep(1:3,itStep) = logMapSO3(T(1:3,1:3)');
    Xstep(4:6,itStep) = -T(1:3,1:3)'*T(1:3,4);
end

% select non-nan values
Xstep = Xstep(:,1:itStep);

% convert the parameters to ones corresponding to the shapes before
% normalization
Xout = Xstep(:,end);
Xout(4:6) = Xout(4:6)*scale;
Xout(4:6) = Xout(4:6)-expMapSO3(Xout(1:3))*meanModel'+meanModel';


for i = 1:size(Xstep,2)
    Xstep(4:6, i) = Xstep(4:6, i)*scale;
    Xstep(4:6, i) = Xstep(4:6, i)-expMapSO3(Xstep(1:3, i))*meanModel'+meanModel';
end

% compute output rotation matrix and translation vector
Rout = expMapSO3(Xout(1:3));
Tout = Xout(4:6);

end




