% this script is called by nms_3Dmodel.mdl
walkType = 1;
% 1: normal walk
% 2: robust walk

switch walkType
    case 1
        load('paramIC_02cm');	% paramIC
        load('param_02cm');  	% param
    case 2
        load('paramIC_10cm');   % paramIC
        load('param_10cm');     % param
end

nms_MechInit;
setInitPar;
setCtrlPar;




