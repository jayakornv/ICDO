%% Load trained model and generate data for test

% load trained ICDO model
load("ICDOmap.mat")

% load data

% get test shapes
pcFile = {'cat1_final.ply','centaur1_final.ply','dog7_final.ply','gorilla0_final.ply','gun0026_final.ply','horse7_final.ply','wolf2_final.ply'};
pcFileDir = fullfile('data','synthetic');
pcList = cell(length(pcFile),1);

for itPcFile = 1:length(pcFile)
    
    % read pc file
    pcList{itPcFile} = pcread(fullfile(pcFileDir,pcFile{itPcFile}));
    pcList{itPcFile} = pcList{itPcFile}.Location;
    
    % normalize
    pcList{itPcFile} = pcList{itPcFile} - mean(pcList{itPcFile});
    pcList{itPcFile} = pcList{itPcFile}/max(abs(pcList{itPcFile}(:)));
end

% set test data generation options
testDataOption = struct();
testDataOption.modelList           = pcList; % list of models
testDataOption.modelNPtRange       = [200 400]; % number of points
testDataOption.angRange            = [0 60]; % rotation
testDataOption.transRange          = [0 0.3]; % translation
testDataOption.noiseSdRange        = [0 0.03]; % gaussian noise
testDataOption.occlu               = [0 0.5]; % occlusion ratio
testDataOption.outl                = [0 0.5]; % outlier ratio

% generate test data
[model,scene, modelClean, R_gt, t_gt] = generate_data(testDataOption);

%% Run ICDO
% Note that the function returns R_out and t_out such that
% scene ~ model * R_out' + t_out'
[R_out, t_out, x_out, X_step] = ICDO_regis(model, scene, ICDOmap, 200, 0.5);

%% Visualize data and registration steps

figure('color','w','Position',[0, 0, 1400, 700])
subplot(1,2,1)
scatter3(scene(:,1), scene(:,2), scene(:,3), 25, 'go', 'filled')
hold on
scatter3(model(:,1), model(:,2), model(:,3), 25, 'bo', 'filled')
legend('Scene','Initial model')
axis equal
title('Scene and initial model')

subplot(1,2,2)
scatter3(scene(:,1), scene(:,2), scene(:,3), 25, 'go', 'filled')
hold on
model_regis = model*R_out' + t_out';
scatter3(model_regis(:,1), model_regis(:,2), model_regis(:,3), 25, 'ro', 'filled')
legend('Scene','Registered model')
axis equal
title('Scene and registered model')

% Visualize step
fig = figure('color','w','Position',[100, 100, 700, 700]);
X_step_init = [zeros(6,1) X_step]; % add init step
for i = 1:size(X_step,2)
    
    % compute rotation matrix and translation vector
    Rt = expMapSO3(X_step_init(1:3,i));
    Tt = X_step_init(4:6,i);
    
    % transform
    model_t = model*Rt' + Tt';
    
    % visualize
    scatter3(scene(:,1), scene(:,2), scene(:,3), 25, 'go', 'filled')
    hold on
    model_regis = model*R_out' + t_out';
    scatter3(model_t(:,1), model_t(:,2), model_t(:,3), 25, 'ro', 'filled')
    hold off
    axis equal
    title(sprintf("Step: %d/%d. Press any key to continue. ", i-1, size(X_step_init,2)-1))
    
    figure(fig);
    pause
end