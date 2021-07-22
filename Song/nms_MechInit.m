% 
% nms_MechInit.m
%   set mechanical parameters of the model
%
% set:
%   segment dynamics
%   muscle-skeleton attachments
%   muscle dynamics
%   ground contact dynamics
%   neural transmission delays
%
% modified from Hartmut Geyer
% October 2006
%
% by Seungmoon Song
% Jan 2014
%

% gravity
g = 9.81;


% ================ %
% SEGMENT DYNAMICS %
% ================ %

% ------------
% foot segment
% ------------

% Note: CS1 is position of fore-foot impact point and contacts 
%       the adjoining ground.

% foot length
L_F = 0.2; %[m]

% distance CS1 to CG (local center of gravity)
D1G_F = 0.14; %[m]

% distance CS1 to C2 (ankle joint)
D12_F = 0.16; %[m]16

% distance CS1 to CS3 (heel pad impact point)
D13_F = 0.20; %[m]20

% foot mass
m_F  = 1.25; %[kg] 1.25

% foot inertia with respect to axis ball-CG-heel (scaled from other papers)
In_F    = 0.005; %[kg*m^2]
Inx_F   = 0.0007;
Iny_F   = 0.005;


% -------------
% shank segment
% -------------

% Note: CS1 is position of ankle joint.

% distance CS1 to CG (local center of gravity)
D1G_S = 0.3; %[m]

% distance CS1 to C2 (knee joint)
D12_S = 0.5; %[m]

% shank mass
m_S  = 3.5; %[kg]

% shank inertia with respect to axis ankle-CG-knee (scaled from other papers)
In_S    = 0.05; %[kg*m^2]
Inx_S   = 0.003;
Iny_S   = 0.05;


% -------------
% thigh segment
% -------------

% Note: CS1 is position of knee joint.

% distance CS1 to CG (local center of gravity)
D1G_T = 0.3; %[m] 

% distance CS1 to C2 (hip joint)
D12_T = 0.5;    %[m]
D12z_T = 0.1;   %[m]

% thigh mass
m_T  = 8.5; %[kg]

% thigh inertia with respect to axis ball-CG-heel (scaled from other papers)
In_T    = 0.15; %[kg*m^2]
Inx_T   = 0.03;
Iny_T   = 0.15;


% -----------------------------
% head-arms-trunk (HAT) segment
% -----------------------------

% HAT length
L_HAT = 0.8; %[m]

% distance hip to center of gravity
D1G_HAT = 0.35; %[m]

% HAT mass
m_HAT = 53.5; %[kg]

% HAT inertia (scaled from other papers)
In_HAT  = 2.5;
Inx_HAT = 1.0;
Iny_HAT = 4.0;


% ----------------------------
% thigh segment pressure sheet
% ----------------------------

% reference compression corresponding to steady-state with HAT mass
DeltaThRef = 5e-3; %[m]

% interaction stiffness
k_pressure = m_HAT * 9.81 / DeltaThRef; %[N/m]

% max relaxation speed (one side)
v_max_pressure = 0.5; %[m/s]

% position in thigh
ThPos = 4/5; % in length of thigh, measured from lower thigh end

% -------------------------------------
% modifications for adding ankle height
% -------------------------------------
ankle_height = 0.08;
D1Gy_F = ankle_height*D1G_F/D12_F;

% shanke
D12_S = D12_S - ankle_height/2;
D1G_S = D1G_S - ankle_height/4;

% thigh
D12_T = D12_T - ankle_height/2; %[m]
D1G_T = D1G_T - ankle_height/4; %[m]

leg_l = [D12_T D12_S];
leg_0 = D12_S + D12_T; % [m] full leg length (from hip to ankle)
    
% -----------------
% soft joint limits
% -----------------

% angles at which soft limits engages
phi12_LowLimit =  70*pi/180; %[rad]
phi12_UpLimit  = 130*pi/180; %[rad]

phi23_UpLimit  = 175*pi/180; %[rad]

phi34_UpLimit  = 230*pi/180; %[rad]

phi34R_LowLimit  = -15*pi/180; %[rad]
phi34R_UpLimit  = 50*pi/180; %[rad]

% soft block reference joint stiffness
c_jointstop     = 0.3 / (pi/180);  %[Nm/rad]

% soft block maximum joint stop relaxation speed
w_max_jointstop = 1 * pi/180; %[rad/s]



% =========================== %
% MUSCLE-SKELETON ATTACHMENTS %
% =========================== %
                         
% hip abductor (HAB)
rHAB      =      0.06; % [m]   constant lever contribution 
phirefHAB = 10*pi/180; % [rad] reference angle at which MTU length equals 
rhoHAB    =       0.7; %       sum of lopt and lslack 

% hip adductor (HAD)
rHAD      =      0.03; % [m]   constant lever contribution 
phirefHAD = 15*pi/180; % [rad] reference angle at which MTU length equals 
rhoHAD    =         1; %       sum of lopt and lslack 
       
% Hip FLexor group attachement
rHFL       =       0.08; % [m]   constant lever contribution 
phirefHFL  = 160*pi/180; % [rad] reference angle at which MTU length equals 
rhoHFL     =        0.5; %       sum of lopt and lslack          

% GLUtei group attachement
rGLU       =       0.08; % [m]   constant lever contribution 
phirefGLU  = 120*pi/180; % [rad] reference angle at which MTU length equals 
rhoGLU     =        0.5; %       sum of lopt and lslack 
                         
% HAMstring group attachement (hip)
rHAMh       = 0.08;         % [m]   constant lever contribution 
phirefHAMh  = 150*pi/180;   % [rad] reference angle at which MTU length equals 
rhoHAMh     = 0.5;          %       sum of lopt and lslack 

% HAMstring group attachement (knee)
rHAMk       = 0.05;         % [m]   constant lever contribution 
phirefHAMk  = 180*pi/180;   % [rad] reference angle at which MTU length equals 
rhoHAMk     = 0.5;          %       sum of lopt and lslack 

% RF group attachement (hip)
rRFh      =       0.08; % [m]   constant lever contribution 
phirefRFh = 170*pi/180; % [rad] reference angle at which MTU length equals 
rhoRFh    =        0.3; %       sum of lopt and lslack 

% RF group attachement (knee)
rRFkmax     = 0.06;         % [m]   maximum lever contribution
rRFkmin     = 0.04;         % [m]   minimum lever contribution
phimaxRFk   = 165*pi/180;   % [rad] angle of maximum lever contribution
phiminRFk   =  45*pi/180;   % [rad] angle of minimum lever contribution
phirefRFk   = 125*pi/180;   % [rad] reference angle at which MTU length equals 
rhoRFk      = 0.5;          %       sum of lopt and lslack 
phiScaleRFk = acos(rRFkmin/rRFkmax)/(phiminRFk-phimaxRFk);

% VAStus group attachement
rVASmax     = 0.06;         % [m]   maximum lever contribution
rVASmin     = 0.04;         % [m]   minimum lever contribution
phimaxVAS   = 165*pi/180;   % [rad] angle of maximum lever contribution
phiminVAS   =  45*pi/180;   % [rad] angle of minimum lever contribution
phirefVAS   = 120*pi/180;   % [rad] reference angle at which MTU length equals 
rhoVAS      = 0.6;          %       sum of lopt and lslack
phiScaleVAS = acos(rVASmin/rVASmax)/(phiminVAS-phimaxVAS);

% BFSH group attachement
rBFSH    	= 0.04;         % [m]   constant lever contribution 
phirefBFSH 	= 160*pi/180;   % [rad] reference angle at which MTU length equals 
rhoBFSH    	= 0.7;          %       sum of lopt and lslack
   
% GAStrocnemius attachement (knee joint)
rGASkmax    = 0.05;         % [m]   maximum lever contribution
rGASkmin    = 0.02;         % [m]   minimum lever contribution
phimaxGASk  = 140*pi/180;   % [rad] angle of maximum lever contribution
phiminGASk  =  45*pi/180;   % [rad] angle of minimum lever contribution
phirefGASk  = 165*pi/180;   % [rad] reference angle at which MTU length equals 
rhoGASk     = 0.7;          %       sum of lopt and lslack 
phiScaleGASk = acos(rGASkmin/rGASkmax)/(phiminGASk-phimaxGASk);

% GAStrocnemius attachement (ankle joint)
rGASamax    = 0.06;         % [m]   maximum lever contribution
rGASamin    = 0.02;         % [m]   minimum lever contribution
phimaxGASa  = 100*pi/180;  	% [rad] angle of maximum lever contribution
phiminGASa  = 180*pi/180; 	% [rad] angle of minimum lever contribution
phirefGASa  =  80*pi/180;  	% [rad] reference angle at which MTU length equals 
rhoGASa     =        0.7; 	%       sum of lopt and lslack 
phiScaleGASa  = acos(rGASamin/rGASamax)/(phiminGASa-phimaxGASa);

% SOLeus attachement
rSOLmax     = 0.06;         % [m]   maximum lever contribution
rSOLmin     = 0.02;         % [m]   minimum lever contribution
phimaxSOL   = 100*pi/180;   % [rad] angle of maximum lever contribution
phiminSOL   = 180*pi/180;   % [rad] angle of minimum lever contribution
phirefSOL   =  90*pi/180;   % [rad] reference angle at which MTU length equals 
rhoSOL      = 0.5;          %       sum of lopt and lslack 
phiScaleSOL = acos(rSOLmin/rSOLmax)/(phiminSOL-phimaxSOL);

% Tibialis Anterior attachement
rTAmax      = 0.04;         % [m]   maximum lever contribution
rTAmin      = 0.01;         % [m]   minimum lever contribution
phimaxTA    =  80*pi/180;   % [rad] angle of maximum lever contribution
phiminTA    = 180*pi/180;   % [rad] angle of minimum lever contribution
phirefTA    = 110*pi/180;   % [rad] reference angle at which MTU length equals 
phiScaleTA  = acos(rTAmin/rTAmax)/(phiminTA-phimaxTA);
rhoTA       = 0.7; 



% =============== %
% MUSCLE DYNAMICS %
% =============== %

% -------------------------------
% shared muscle tendon parameters
% -------------------------------

% excitation-contraction coupling
preA =  0.01; %[] preactivation
tau  =  0.01; %[s] delay time constant

% contractile element (CE) force-length relationship
w    =   0.56; %[lopt] width
c    =   0.05; %[]; remaining force at +/- width

% CE force-velocity relationship
N    =   1.5; %[Fmax] eccentric force enhancement
K    =     5; %[] shape factor

% Series elastic element (SE) force-length relationship
eref =  0.04; %[lslack] tendon reference strain


% --------------------------
% muscle-specific parameters
% --------------------------

% hip abductor (HAB)
FmaxHAB    =     3000; % maximum isometric force [N]
loptHAB    =     0.09; % optimum fiber length CE [m]
vmaxHAB    =       12; % maximum contraction velocity [lopt/s]
lslackHAB  =     0.07; % tendon slack length [m]

% hip adductor (HAD)
FmaxHAD    =     4500; % maximum isometric force [N]
loptHAD    =     0.10; % optimum fiber length CE [m]
vmaxHAD    =       12; % maximum contraction velocity [lopt/s]
lslackHAD  =     0.18; % tendon slack length [m]

% hip flexor muscles
FmaxHFL   = 2000; % maximum isometric force [N]
loptHFL   = 0.11; % optimum fiber length CE [m]
vmaxHFL   =   12; % maximum contraction velocity [lopt/s]
lslackHFL = 0.10; % tendon slack length [m]

% glutei muscles
FmaxGLU   = 1500; % maximum isometric force [N]
loptGLU   = 0.11; % optimum fiber length CE [m]
vmaxGLU   =   12; % maximum contraction velocity [lopt/s]
lslackGLU = 0.13; % tendon slack length [m]

% hamstring muscles
FmaxHAM   = 3000; % maximum isometric force [N]
loptHAM   = 0.10; % optimum fiber length CE [m]
vmaxHAM   =   12; % maximum contraction velocity [lopt/s]
lslackHAM = 0.31; % tendon slack length [m]

% rectus femoris muscles
FmaxRF   = 1200; % %850 maximum isometric force [N]
loptRF   = 0.08; % optimum fiber length CE [m]
vmaxRF   =   12; % maximum contraction velocity [lopt/s]
lslackRF = 0.35; % tendon slack length [m]

% vasti muscles
FmaxVAS     = 6000; % maximum isometric force [N]
loptVAS     = 0.08; % optimum fiber length CE [m]
vmaxVAS     =   12; % maximum contraction velocity [lopt/s]
lslackVAS   = 0.23; % tendon slack length [m]

% BFSH
FmaxBFSH	=  350; % maximum isometric force [N]
loptBFSH    = 0.12; % optimum fiber length CE [m]
vmaxBFSH    =   12; %6 % maximum contraction velocity [lopt/s]
lslackBFSH  = 0.10; % tendon slack length [m]

% gastrocnemius muscle
FmaxGAS    = 1500; % maximum isometric force [N]
loptGAS    = 0.05; % optimum fiber length CE [m]
vmaxGAS    =   12; % maximum contraction velocity [lopt/s]
lslackGAS  = 0.40; % tendon slack length [m]

% soleus muscle
FmaxSOL    = 4000; % maximum isometric force [N]
loptSOL    = 0.04; % optimum fiber length CE [m]
vmaxSOL    =    6; % maximum contraction velocity [lopt/s]
lslackSOL  = 0.26; % tendon slack length [m]

% tibialis anterior
FmaxTA     =  800; % maximum isometric force [N]
loptTA     = 0.06; % optimum fiber length CE [m]
vmaxTA     =   12; % maximum contraction velocity [lopt/s]
lslackTA   = 0.24; % tendon slack length [m]



% ======================= %
% GROUNG CONTACT DYNAMICS %
% ======================= %

% ------------------
% vertical component
% ------------------

% stiffness of vertical ground interaction
k_gn = (2*(m_F+m_S+m_T)+m_HAT)*g/0.01; %[N/m]

% max relaxation speed of vertical ground interaction
v_gn_max = 0.03; %[m/s]


% --------------------
% horizontal component
% --------------------

% sliding friction coefficient
mu_slide = 0.8;

% sliding to stiction transition velocity limit
vLimit = 0.01; %[m/s]

% stiffness of horizontal ground stiction
k_gt = (2*(m_F+m_S+m_T)+m_HAT)*g/0.1; %[N/m] 0.01

% max relaxation speed of horizontal ground stiction
v_gt_max = 0.03; %[m/s] 0.03

% stiction to sliding transition coefficient
mu_stick = 0.9;


% --------------
% for 3D contact
% --------------

n_Cnt = 8;
d_fBall = 0.1;
d_fHeel = 0.05;

k_gn    = k_gn/2;
k_gt    = k_gt/2;



% ========================== %
% NEURAL TRAJSMISSION DELAYS %
% ========================== %

LongLoopDelay	= 0.030;    % [s] additional to spinal reflexes
LongDelay       = 0.020/2;  % [s] ankle joint muscles
MidDelay        = 0.010/2;  % [s] knee joint muscles
ShortDelay      = 0.005/2;  % [s] hip joint muscles
MinDelay        = 0.001/2;  % [s] between neurons in the spinal cord



