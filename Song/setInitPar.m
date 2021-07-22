%
% setInitPar.m
%   set initial parameters
%   given: paramIC
%
% by Seungmoon Song
% Jan 2014
%

vx0              	= paramIC(1)  	*1.3; %[m/s] 
Lphi120             = paramIC(2)   	*100*pi/180; %[rad]
Lphi230             = paramIC(3)   	*180*pi/180; %[rad]
Lphi340             = paramIC(4) 	*165*pi/180; %[rad]
Rphi120             = paramIC(5)  	*90*pi/180; %[rad]
Rphi230             = paramIC(6)  	*165*pi/180; %[rad]
Rphi340             = paramIC(7)   	*200*pi/180; %[rad]
Lphi340R            = paramIC(8)*(-1)*pi/180;
Rphi340R            = paramIC(9)*(-1)*pi/180;

vy0 = paramIC(10)*.2;

x0      = .2;
y0      = D12z_T;
z0      = paramIC(11)      *.01;
yaw0    = 0*pi/180;
roll0   = (-1)*pi/180;

init_a_tgt  = 70*pi/180;
init_a_tgtR = 90*pi/180;