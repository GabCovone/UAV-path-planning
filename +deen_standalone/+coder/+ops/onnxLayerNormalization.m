function [Y, numDimsY] = onnxLayerNormalization(X_, scale_, offset_, axis_, epsilon_, numDimsX_)
%onnxLayerNormalization Normalizes mini-batches of data over the normalized shape.
%#codegen

% Copyright 2024 The MathWorks, Inc.

X           = deen_standalone.coder.ops.extractIfDlarray(X_);
scale       = deen_standalone.coder.ops.extractIfDlarray(scale_);
offset      = deen_standalone.coder.ops.extractIfDlarray(offset_);
axis        = deen_standalone.coder.ops.extractIfDlarray(axis_);
epsilon     = deen_standalone.coder.ops.extractIfDlarray(epsilon_);
numDimsX    = deen_standalone.coder.ops.extractIfDlarray(numDimsX_);

% axis is in forward ONNX format. Convert axis to reverse ONNX format. 
if axis < 0
    axis = -axis;
else
    axis = numDimsX - axis; 
end

epsilon = double(epsilon);
% Epsilon must be non-negative.
if(epsilon <= 0)   
    epsilon = 1e-5;
end

dimension = 1 : axis;

% stage 1: standarization
XMean = mean(X, dimension);
D = X - XMean;
DD = D.^2; 
XVar = mean(DD, dimension);
XVarEps = XVar + epsilon;
StdDev = sqrt(XVarEps);
InvStdDev  = 1 ./ StdDev;
Normalized = D .* InvStdDev;

% stage 2
NormalizedScaled = Normalized .* scale;

if ~isempty(offset)
    Y = NormalizedScaled + offset;
else
    Y = NormalizedScaled;
end
numDimsY = numDimsX;
end