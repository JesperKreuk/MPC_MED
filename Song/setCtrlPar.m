%
% setCtrlPar.m
%   set control parameters
% 
%   given: param
%
%   set:
%       CtrlPar_theta0
%       CtrlPar_FP
%       CtrlPar_LL
%       CtrlPar_St
%       CtrlPar_Sw
%       CtrlPar_Tr
%       CtrlPar_FP_R
%       CtrlPar_St_R
%       CtrlPar_Sw_R
%       CtrlPar_Tr_R
%
% by Seungmoon Song
% Jan 2014
%

l_BlP = 1;
l_FPP = 3;
l_LLP = 1;
l_StP = 30;
l_SwP = 27;
l_TrP = 2;

l_2DCtrl    = l_BlP + l_FPP + l_LLP + l_StP + l_SwP + l_TrP;
l_3DCtrl    = 18;

param2D     = param(1                       : l_2DCtrl);
param3D     = param(l_2DCtrl+1              : l_2DCtrl+l_3DCtrl);


% ==================================== %
% SET LATERAL PLANE CONTROL PARAMETERS %
% ==================================== %

% --------------------
% higher layer control
% --------------------

% foot placement Ctrl %
R_hipR0             = param3D(1)        *(-1)   *pi/180; 
R_hipR_cd           = param3D(2)        *10     *pi/180;    % [rad/m]
R_hipR_cv           = param3D(3)        *10     *pi/180;    % [rad*s/m]

CtrlPar_FP_R = [R_hipR0 R_hipR_cd R_hipR_cv];


% --------------
% stance control
% --------------

% 0: prestimulations
St_PreS_HAB       	= param3D(4)       *0.01;
St_PreS_HAD         = param3D(5)       *0.01;

% M1: realize compliant leg
R_St_FGain_HAB    	= param3D(6)        *.45/FmaxHAB;

% M3: balance trunk
R_St_GainTh_HAB     = param3D(7)        *2.4;
R_St_GainDTh_HAB    = param3D(8)        *.4;
R_St_GainTh_HAD     = param3D(9)        *.5;
R_St_GainDTh_HAD    = param3D(10)      	*.55;

% M4: compensate swing leg
R_St_SGainCHAB_HAB  = param3D(11)    	*1.5;
R_St_SGainCHAD_HAD  = param3D(12)      	*.4;

R_StPreStim = [St_PreS_HAB St_PreS_HAD];
R_St_M1     = [R_St_FGain_HAB];
R_St_M3     = [R_St_GainTh_HAB R_St_GainDTh_HAB R_St_GainTh_HAD R_St_GainDTh_HAD];
R_St_M4     = [R_St_SGainCHAB_HAB R_St_SGainCHAD_HAD];

CtrlPar_St_R = [R_StPreStim R_St_M1 R_St_M3 R_St_M4];

% indexes of control parameters
R_St_i = 0;
R_parSt_PreS_i  = R_St_i + 1	: R_St_i + length(R_StPreStim);
R_St_i = R_St_i + length(R_StPreStim);
R_parSt_M1_i  	= R_St_i + 1	: R_St_i + length(R_St_M1);
R_St_i = R_St_i + length(R_St_M1);
R_parSt_M3_i  	= R_St_i + 1	: R_St_i + length(R_St_M3);
R_St_i = R_St_i + length(R_St_M3);
R_parSt_M4_i  	= R_St_i + 1	: R_St_i + length(R_St_M4);
R_St_i = R_St_i + length(R_St_M4); % St_i is used in the Simulink file


% -------------
% swing control
% -------------

% constants (measurment parameters)
R_Sw_a2lopt_HAB     = 0.5442*rHAB*rhoHAB/loptHAB;
R_Sw_aRef_HAB       = 0.8628*(pi/2 - phirefHAB);
R_Sw_a2lopt_HAD     = 0.6798*rHAD*rhoHAD/loptHAD;
R_Sw_aRef_HAD       = 0.8053*(pi/2 - phirefHAD);

% 0: prestimulations
Sw_PreS_HAB       	= param3D(13)       *0.01;
Sw_PreS_HAD         = param3D(14)       *0.01;

% M6: swing hip
R_Sw_LGain_HAB      = param3D(15)       *1/R_Sw_a2lopt_HAB;
R_Sw_LGain_HAD      = param3D(16)       *1/R_Sw_a2lopt_HAD;

R_SwPreStim = [Sw_PreS_HAB Sw_PreS_HAD];
R_Sw_M6     = [R_Sw_a2lopt_HAB R_Sw_aRef_HAB R_Sw_LGain_HAB R_Sw_a2lopt_HAD R_Sw_aRef_HAD R_Sw_LGain_HAD];

CtrlPar_Sw_R = [R_SwPreStim R_Sw_M6];

% indexes of control parameters
R_Sw_i = 0;
R_parSw_PreS_i  = R_Sw_i + 1	: R_Sw_i + length(R_SwPreStim);
R_Sw_i = R_Sw_i + length(R_SwPreStim);
R_parSw_M6_i  	= R_Sw_i + 1	: R_Sw_i + length(R_Sw_M6);
R_Sw_i = R_Sw_i + length(R_Sw_M6); % St_i is used in the Simulink file


% ----------------------------------
% stance -> swing transition control
% ----------------------------------

R_Tr_St_sup         = param3D(17)       *1;       
R_Tr_Sw             = param3D(18)       *1;

CtrlPar_Tr_R = [R_Tr_St_sup R_Tr_Sw];



% ===================================== %
% SET SAGITTAL PLANE CONTROL PARAMETERS %
% ===================================== %

parBlCtrl   = param2D(1                                                   : l_BlP);
parFPCtrl   = param2D(l_BlP + 1                                           : l_BlP + l_FPP);
parLLCtrl   = param2D(l_BlP + l_FPP + 1                                   : l_BlP + l_FPP + l_LLP);
parStCtrl   = param2D(l_BlP + l_FPP + l_LLP + 1                           : l_BlP + l_FPP + l_LLP + l_StP);
parSwCtrl   = param2D(l_BlP + l_FPP + l_LLP + l_StP + 1                   : l_BlP + l_FPP + l_LLP + l_StP + l_SwP);
parTrCtrl   = param2D(l_BlP + l_FPP + l_LLP + l_StP + l_SwP + 1           : l_BlP + l_FPP + l_LLP + l_StP + l_SwP + l_TrP);


% --------------------
% higher layer control
% --------------------

% trunk balance Ctrl
CtrlPar_theta0      = parBlCtrl(1)      *1*pi/180;


% foot placement Ctrl (alpha Ctrl)

alpha0              = parFPCtrl(1)      *70 *pi/180; 
alpha_cd            = parFPCtrl(2)      *(-5) *pi/180; % [rad/m]
alpha_cv            = parFPCtrl(3)      *(-5) *pi/180; % [rad*s/m]

CtrlPar_FP = [alpha0 alpha_cd alpha_cv];

% swing leg length Ctrl
l_clr  	= parLLCtrl(1)*0.85;

CtrlPar_LL = l_clr;


% ------------------
% stance leg control
% ------------------

% 0: prestimulations
St_PreSHFL          = parStCtrl(1)      *0.05; %[]
St_PreSGLU          = parStCtrl(2)      *0.05; %[]
St_PreSHAM          = parStCtrl(3)      *0.05; %[]
St_PreSRF           = parStCtrl(4)      *0.01;
St_PreSVAS          = parStCtrl(5)      *0.08; %[]
St_PreSBFSH         = parStCtrl(6)      *0.02;
St_PreSGAS          = parStCtrl(7)      *0.01;
St_PreSSOL          = parStCtrl(8)      *0.01;
St_PreSTA           = parStCtrl(9)      *0.01;

% M1: realize compliant leg
St_FGain_GLU      	= parStCtrl(10)     *1.0/FmaxGLU;
St_FGain_VAS     	= parStCtrl(11)     *1.2/FmaxVAS;
St_FGain_SOL     	= parStCtrl(12)     *1.2/FmaxSOL;

% M2: prevent knee overextension
St_FGain_HAM     	= parStCtrl(13)     *1.0/FmaxHAM;
St_loffBFSH_VAS     = parStCtrl(14)     *2;
St_LGainBFSH_VAS    = parStCtrl(15)     *2;
St_loffBFSH         = parStCtrl(16)     *1.1;
St_LGain_BFSH       = parStCtrl(17)     *2;
St_FGain_GAS        = parStCtrl(18)     *1.22/FmaxGAS;

% M3: balance trunk
St_GainTh_HFL       = parStCtrl(19)     *1;
St_GainDTh_HFL      = parStCtrl(20)     *0.3;
St_GainTh_GLU       = parStCtrl(21)     *0.5;
St_GainDTh_GLU      = parStCtrl(22)     *0.1;
St_SGainGLU_HAM     = parStCtrl(23)     *1;

% M4: compensate swing leg
St_SGainCGLU_HFL    = parStCtrl(24)     *.1;
St_SGainCHAM_HFL    = parStCtrl(25)     *.1;
St_SGainCHFL_GLU    = parStCtrl(26)     *.1;
St_SGainCRF_GLU     = parStCtrl(27)     *.1;
% St_SGainGLU_HAM     = St_SGainGLU_HAM;

% M5: flex ankle
St_LceOffTA         = parStCtrl(28)     *(1-0.65*w); %[loptTA]
St_LGainTA          = parStCtrl(29)     *1.1;
St_FGainSOL_TA      = parStCtrl(30)     *0.4/FmaxSOL;

% pass control parameters to Simulink
StPreStim           =   ...
    [St_PreSHFL St_PreSGLU St_PreSHAM St_PreSRF St_PreSVAS ...
    St_PreSBFSH St_PreSGAS St_PreSSOL St_PreSTA];

St_M1       = [St_FGain_GLU St_FGain_VAS St_FGain_SOL];
St_M2       = [St_FGain_HAM St_loffBFSH_VAS St_LGainBFSH_VAS St_loffBFSH St_LGain_BFSH St_FGain_GAS];
St_M3       = [St_GainTh_HFL St_GainDTh_HFL St_GainTh_GLU St_GainDTh_GLU St_SGainGLU_HAM];
St_M4       = [St_SGainCGLU_HFL St_SGainCHAM_HFL St_SGainCHFL_GLU St_SGainCRF_GLU St_SGainGLU_HAM];
St_M5       = [St_LceOffTA St_LGainTA St_FGainSOL_TA];


CtrlPar_St = [StPreStim St_M1 St_M2 St_M3 St_M4 St_M5];

% indexes of control parameters
St_i = 0;
parSt_PreS_i    = St_i + 1	: St_i + length(StPreStim);
St_i = St_i + length(StPreStim);
parSt_M1_i   	= St_i + 1	: St_i + length(St_M1);
St_i = St_i + length(St_M1);
parSt_M2_i      = St_i + 1	: St_i + length(St_M2);
St_i = St_i + length(St_M2);
parSt_M3_i      = St_i + 1	: St_i + length(St_M3);
St_i = St_i + length(St_M3);
parSt_M4_i      = St_i + 1	: St_i + length(St_M4);
St_i = St_i + length(St_M4);
parSt_M5_i      = St_i + 1	: St_i + length(St_M5);
St_i = St_i + length(St_M5); % St_i is used in the Simulink file


% -----------------
% swing leg control
% -----------------

% constants (measurment parameters)
Sw_phi2lopt_BFSH    = 0.8520*rBFSH*rhoBFSH/loptBFSH;
Sw_phiRef_BFSH      = 1.0557*phirefBFSH;
Sw_a2lopt_RF        = 0.7999*rRFh*rhoRFh/loptRF;
Sw_aRef_RF          = 1.2605*(phirefRFh-phirefRFk/2);
Sw_a2lopt_HAM       = 0.8178*rHAMh*rhoHAMh/loptHAM;
Sw_aRef_HAM         = 0.7239*(phirefHAMh-phirefHAMk/2);
Sw_phi2lopt_HFL     = 1.0045*rHFL*rhoHFL/loptHFL;
Sw_phiRef_HFL       = 1.0170*phirefHFL;
Sw_phi2lopt_GLU     = 0.9969*rGLU*rhoGLU/loptGLU;
Sw_phiRef_GLU       = 0.9876*phirefGLU;

% 0: prestimulations
Sw_PreSHFL          = parSwCtrl(1)      *0.01;
Sw_PreSGLU          = parSwCtrl(2)      *0.01;
Sw_PreSHAM          = parSwCtrl(3)      *0.01;
Sw_PreSRF           = parSwCtrl(4)      *0.01;
Sw_PreSVAS          = parSwCtrl(5)      *0.01;
Sw_PreSBFSH         = parSwCtrl(6)      *0.02;
Sw_PreSGAS          = parSwCtrl(7)      *0.01;
Sw_PreSSOL          = parSwCtrl(8)      *0.01;
Sw_PreSTA           = parSwCtrl(9)      *0.01;

% swing phase
Sw_a_del            = parSwCtrl(10)     *12*pi/180;

% swing Ctrl (hip)
Sw_LGainRF_HFL      = parSwCtrl(11)     *1/Sw_a2lopt_RF;
Sw_VGainRF_HFL      = parSwCtrl(12)     *.5;
Sw_LGainHAM_GLU     = parSwCtrl(13)     *0.5/Sw_a2lopt_HAM;
Sw_VGainHAM_GLU     = parSwCtrl(14)     *.5;

% swing Ctrl (ankle)
Sw_LGainTA_TA       = parSwCtrl(15)     *1.1;
Sw_LceOffTA         = parSwCtrl(16)     *(1-0.65*w);

% swing Ctrl (knee_i)
Sw_VGainRF_BFSH   	= parSwCtrl(17)     *0.4*loptRF/(rRFh*rhoRFh);

% swing Ctrl (knee_ii)
Sw_VGainVAS_RF  	= parSwCtrl(18)     *0.08*loptBFSH/(rBFSH*rhoBFSH);
Sw_VGainBFSH     	= parSwCtrl(19)     *2.5/Sw_a2lopt_RF;

% swing Ctrl (knee_iii)
Sw_LGainHAM         = parSwCtrl(20)     *2/Sw_a2lopt_HAM;
Sw_SGainHAM_BFSH   	= parSwCtrl(21)     *6;
Sw_SGainHAM_GAS     = parSwCtrl(22)     *2;
Sw_SHAM_thr         = parSwCtrl(23)     *0.65;

% swing Ctrl (stance preparation)
Sw_LGainHFL         = parSwCtrl(24)     *0.4;
Sw_LGainGLU         = parSwCtrl(25)     *0.4;
Sw_loffVAS          = parSwCtrl(26)     *0.1;
Sw_LGainVAS         = parSwCtrl(27)     *0.3;

% pass control parameters to Simulink
SwPreStim       =   ...
    [Sw_PreSHFL Sw_PreSGLU Sw_PreSHAM Sw_PreSRF Sw_PreSVAS ...
    Sw_PreSBFSH Sw_PreSGAS Sw_PreSSOL Sw_PreSTA];

SwPhasePari     = ...
    [Sw_phi2lopt_BFSH Sw_phiRef_BFSH];

SwPhaseParii    = ...
    [Sw_aRef_RF Sw_a2lopt_RF Sw_a_del Sw_aRef_HAM Sw_a2lopt_HAM];
                
SwCtrlPar_H      = ...
    [Sw_a2lopt_RF Sw_aRef_RF Sw_LGainRF_HFL Sw_VGainRF_HFL Sw_a2lopt_HAM ...
    Sw_a_del Sw_aRef_HAM Sw_LGainHAM_GLU Sw_VGainHAM_GLU];

SwCtrlPar_A     = [Sw_LGainTA_TA Sw_LceOffTA];

SwCtrlPar_Ki    = Sw_VGainRF_BFSH;

SwCtrlPar_Kii   = ...
    [Sw_VGainVAS_RF Sw_VGainBFSH Sw_phi2lopt_BFSH Sw_a2lopt_RF Sw_aRef_RF];
 
SwCtrlPar_Kiii  = ...
    [Sw_a_del Sw_aRef_HAM Sw_a2lopt_HAM Sw_LGainHAM Sw_SHAM_thr ...
    Sw_SGainHAM_BFSH Sw_SGainHAM_GAS];
     
SwCtrlPar_StP   = ...
    [Sw_phiRef_HFL Sw_phi2lopt_HFL Sw_LGainHFL Sw_phiRef_GLU Sw_phi2lopt_GLU ...
    Sw_LGainGLU Sw_loffVAS Sw_LGainVAS];

CtrlPar_Sw       =   ...
    [SwPreStim SwPhasePari SwPhaseParii SwCtrlPar_H SwCtrlPar_A ...
    SwCtrlPar_Ki SwCtrlPar_Kii SwCtrlPar_Kiii SwCtrlPar_StP];

% indexes of control parameters
Sw_i = 0;
parSwPreS_i     = Sw_i + 1	: Sw_i + length(SwPreStim);
Sw_i = Sw_i + length(SwPreStim);
parSwPhi_i      = Sw_i + 1	: Sw_i + length(SwPhasePari);
Sw_i = Sw_i + length(SwPhasePari);
parSwPhii_i     = Sw_i + 1	: Sw_i + length(SwPhaseParii);
Sw_i = Sw_i + length(SwPhaseParii);
parSwH_i        = Sw_i + 1	: Sw_i + length(SwCtrlPar_H);
Sw_i = Sw_i + length(SwCtrlPar_H);
parSwA_i        = Sw_i + 1	: Sw_i + length(SwCtrlPar_A);
Sw_i = Sw_i + length(SwCtrlPar_A);
parSwKi_i       = Sw_i + 1	: Sw_i + length(SwCtrlPar_Ki);
Sw_i = Sw_i + length(SwCtrlPar_Ki);    
parSwKii_i      = Sw_i + 1	: Sw_i + length(SwCtrlPar_Kii);
Sw_i = Sw_i + length(SwCtrlPar_Kii);
parSwKiii_i     = Sw_i + 1	: Sw_i + length(SwCtrlPar_Kiii);
Sw_i = Sw_i + length(SwCtrlPar_Kiii);
parSwKiv_i      = Sw_i + 1	: Sw_i + length(SwCtrlPar_StP);
Sw_i = Sw_i + length(SwCtrlPar_StP);


% ----------------------------------
% stance -> swing transition control
% ----------------------------------

Tr_St_sup           = parTrCtrl(1)    *1;
Tr_Sw               = parTrCtrl(2)    *1;

CtrlPar_Tr = [Tr_St_sup Tr_Sw];