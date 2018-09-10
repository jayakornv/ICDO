% demo code from training and running ICDO

disp("Training may take a long time (> 1 hr).")
disp("You can reduce the number of training samples to reduce the time.")

%% Training

% Reading of the model shapes
pcFileDir = fullfile('data','synthetic'); % folder to training ply files
pcFileTrain = {'bunny.ply','armadillo.ply','chef.ply','chic.ply','para.ply','rhino.ply','trex.ply'}; % list training files
pcList = cell(length(pcFileTrain),1);

for itPcFile = 1:length(pcFileTrain)
    
    % read pc file
    pcList{itPcFile} = pcread(fullfile(pcFileDir,pcFileTrain{itPcFile}));
    pcList{itPcFile} = pcList{itPcFile}.Location;
    
    % normalize
    pcList{itPcFile} = pcList{itPcFile} - mean(pcList{itPcFile});
    pcList{itPcFile} = pcList{itPcFile}/max(abs(pcList{itPcFile}(:)));
end

% training parameter setting
trainDataOption                     = struct();
trainDataOption.nTrain              = 1e5; % number of training samples
trainDataOption.modelList           = pcList; % list models used for training 
trainDataOption.modelNPtRange       = [200 400]; % range of number of points in each sample
trainDataOption.angRange            = [0 85]; % range of relative angles
trainDataOption.transRange          = [0 0.2]; % range of translation
trainDataOption.noiseSdRange        = [0 0.03]; % range of noise
trainDataOption.occlu               = [0 0.3]; % range occlusion ratio
trainDataOption.outl                = [0 0.]; % ramge outlier

% generate training data
trainData = struct();
for itTrain = 1:trainDataOption.nTrain
    
    % print progress
    if mod(itTrain,2000) == 0
        fprintf('Generating sample: %d/%d\n',itTrain,trainDataOption.nTrain)
    end
    
    % generate and save
    [modelTmp,sceneTmp, modelCleanTmp, Rtmp, Ttmp] = generate_data(trainDataOption, 'training');
    trainData.model{itTrain} = modelTmp;
    trainData.scene{itTrain} = sceneTmp;
    trainData.xGT(:,itTrain) = [logMapSO3(Rtmp);Ttmp(:)]; % target parameters
end

%% DO training

% Training parameters
trainMapOption = struct();
trainMapOption.tol              = 1e-10; % tolerance for termination
trainMapOption.nMap             = 20;  % maximum number of maps
trainMapOption.lambdaRidge      = 1e-8; % lambda for ridge regression

% ICDO parameters
ICDOParams            = struct();
ICDOParams.radius     = 3; % maximum radius
ICDOParams.nBox       = 100; % number of discretization boxes
ICDOParams.alpha      = 1.15; % alpha

% Train
tStart              = tic;
ICDOmap             = train_maps(trainData, ICDOParams, trainMapOption);
ICDOmap.trainTime   = toc(tStart);




