function h = compute_h_single(model, scene, radius, nBox, iter, alpha)
%COMPUTE_H_SINGLE Compute feature for a single pair of data. Specifically,
%the code computes the feature function h in Eq. (23) in the ICDO paper
%with addition steps described in the implementation details section. It
%employs quite a few matrix index and ordering tricks to make computation
%faster.
%

% Consider each model point as m_i. This line computes -[m_i]_x (negative 
% cross product matrix of m_i) and vectorizes it into a row vector. This is
% done as part of derivative computation (See eq. (16) in the ICDO paper).
% CPM = cross product matrix
model_CPM = -([0 1 -1 -1 0 1 1 -1 0].*model(:,[1 3 2 3 1 1 2 1 1]))'; 

% compute difference vector between all pairs of points in model and scene
D = permute(scene,[1 3 2])-permute(model,[3 1 2]);

% compute norms of vectors in D and normalize into unit vectors
if size(D,1)*size(D,2) < 2e5
    V = sum(D.^2,3);
else
    V = pwSqDist(double(scene)',double(model)'); 
end
V(V<=0) = inf;  % set any distance <= 0 to inf to eliminate later since 
                % they don't contribute to gradient
G = sqrt(V);    % sqrt into norms
D = D./G;       % normalize to unit vectors

% compute the box indices that each value will be discretized to
BoxInd = floor((nBox/radius*alpha^(iter-1))*G)+1;

% remove invalid indices and compute the column coordinate of eachvalid 
% index 
valInd = find(BoxInd(:) <= nBox);
valIndCol = ceil(valInd/size(G,1));

% reshape D
D_reshape = reshape(D,numel(BoxInd),3);
D_reshape = D_reshape(valInd,:);

% compute w_ij for all i,j in eq. (16)
model_CPM_reshape = reshape(model_CPM',size(model_CPM,2),3,3);
W = model_CPM_reshape(valIndCol,:,:);
W = sum(W.*D_reshape,2);
W = reshape(W,length(valIndCol),3);
W = [W D_reshape]; %%#ok<AGROW>

% compute the index of h in E. (23) where each value in w_ij will be set to 
validBox = BoxInd(valInd)+(0:nBox:5*nBox);

% compute h by putting values in the corresponding indices
h = accumarray(validBox(:),W(:),[nBox*6,1]);

% normalize by number of pairs
h = h/numel(G);
end
