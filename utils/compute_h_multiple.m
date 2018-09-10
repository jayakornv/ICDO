function h = compute_h_multiple(models, scenes, radius, nBox, iter, alpha)
%COMPUTE_H_MULTIPLE Compute the features for all pairs of data

% Allocate space
h = zeros(6*nBox, numel(models));

% Loop over each data pair
for itData = 1:numel(models)
    h(:,itData) = compute_h_single(models{itData}, scenes{itData}, radius, nBox, iter, alpha);
end

end

