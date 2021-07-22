function buildMexFile(params)
%% Validate model on other dataset
%%% Validation of prediction on the full dataset of the nonlinear model

% loadGreyEstData
% datasetVal = 11;    
% skipSwingPercentageVal = 0;
% 
% GaitPhaseData       = simout.GaitPhaseData;
% [LSwingIndices, RSwingIndices] = findSwingIndices(GaitPhaseData);
% 
% % Find corresponding indices
% indices = cell2mat(RSwingIndices(datasetVal));
% 
% [Rdatax, Rdatau, Ndata] = calculateSensorData(LAnklePosxy,...
%                 RAnklePosxy,LHipPosxy,RHipPosxy,hipCorrection,indices);
% 
% RdataxRes = Rdatax(1:10:end,:).';
x0 = zeros(4,1);

%%
codegen NMPC -args {x0, 1, params}
end
% %%
% tic
% uopt = NMPC_mex(x0,params)
% toc
% %% 
% tic
% uopt = NMPC(x0,params)
% tocc