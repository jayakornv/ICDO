function [scale] = calScaleFactor(model)
    scale = mean(svd(model))/sqrt(size(model,1));
end

