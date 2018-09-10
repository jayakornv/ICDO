function [ICDOmap] = train_maps(trainData, DOParams, trainMapOption)
    
    disp('Training maps');
    
    % Parameter initalisation
    errVal  = inf;
    lambda  = trainMapOption.lambdaRidge;
    itMap   = 0;
    
    % extract info from options
    xGT     = trainData.xGT;
    
    model   = trainData.model;
    scene   = trainData.scene;
    
    nData = length(model);
    radius = DOParams.radius;
    nBox = DOParams.nBox;
    alpha = DOParams.alpha;
    
    % normalize cloud
    scaleFactor = cellfun(@(x) calScaleFactor(x),model);
    scaleFactor = scaleFactor(:)';
    for itTrain = 1:length(model)
        model{itTrain} = model{itTrain}/scaleFactor(itTrain);
        scene{itTrain} = scene{itTrain}/scaleFactor(itTrain);
    end
    xGT(4:6,:) = xGT(4:6,:)./scaleFactor;
    
    % training iteration
    while errVal > trainMapOption.tol && itMap < trainMapOption.nMap
                
        itMap = itMap + 1;
        fprintf('Training map: %d\n',itMap);
        
        % extract feature
        tStart = tic;
        [H] = compute_h_multiple(model, scene, radius, nBox, itMap, alpha);
        
        % linear regression of each rotation parameter
        Dtmp = nan(6,nBox);
        for itDB = 1:6
            Htmp = H((itDB-1)*nBox+(1:nBox),:);
            Dtmp(itDB,:) = (xGT(itDB,:)*Htmp')/((Htmp*Htmp')/nData+lambda*eye(size(Htmp,1)))/nData;
        end
        
        % compose the map of each parameter back into a single map
        D = blkdiag(Dtmp(1,:),Dtmp(2,:),Dtmp(3,:),Dtmp(4,:),Dtmp(5,:),Dtmp(6,:));
        
        % compute the update parameters
        updateMat = D*H;
        
        % update the scene shapes and parameters
        for itInst = 1:length(model)
            
            % get transformation parameters
            Rtmp = expMapSO3(updateMat(1:3,itInst));
            ttmp = updateMat(4:6,itInst);
            
            % transform the model in inverse composition fashion
            scene{itInst} = (scene{itInst}-ttmp')*Rtmp;
            
            % compute the difference between original ground truths and 
            % current transformation parameters as new ground truths
            Rgttmp = expMapSO3(xGT(1:3,itInst));
            tgttmp = xGT(4:6,itInst);
            xGT(1:3,itInst) = logMapSO3(Rtmp'*Rgttmp);
            xGT(4:6,itInst) = Rtmp'*(tgttmp-ttmp);
        end
        
        
        % add small perturbation translation
        deltaT = normc(randn(3,nData)).*randn(1,nData)*0.1;
        xGT(4:6,:) = xGT(4:6,:)+deltaT;
        for itInst = 1:length(model)
            scene{itInst} = scene{itInst}+deltaT(:,itInst)';
        end
        
        % add snall perturbation rotation
        for itData = 1:nData
            Rtmp = genRandomRotationAngle(10*randn());
            scene{itData} = (Rtmp*scene{itData}')';
            xGT(1:3,itData) = logMapSO3(Rtmp*expMapSO3(xGT(1:3,itData)));
            xGT(4:6,itData) = Rtmp*xGT(4:6,itData);
        end
                
        % Store iteration results
        Dmap(:,:,itMap) = D;
        
        % Error update
        errVal = norm(xGT,'fro')^2/nData;
        fprintf('Map: %d, Time: %f, Err: %f\n', itMap, toc(tStart), errVal)
    end
    
    % compose the map
    ICDOmap         = struct();
    ICDOmap.D       = Dmap;
    ICDOmap.radius  = radius;
    ICDOmap.nBox    = nBox;
    ICDOmap.alpha   = alpha;
    
end