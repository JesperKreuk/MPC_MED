clear all
close all
clc

% This script will test the MLD system that was build by doing the
% following tests:

% Test 1 
% xP = 0 (so there is no prosthetic swing)
% The following should hold in all cases:
%       x(k+1) = x(k) without any violated constraints
%       (without any violated constraints)
% * Test 1.1: xbar = 0, xp = 0, u = 0 
% * Test 1.2: xbar = 0, xp = 0, u = -1
% * Test 1.3: xbar =/= 0, xp = 0, u =0, th1 + th2 < 0 
% * Test 1.4: xbar =/= 0, xp = 0, u =10, th1 + th2 < 0 
% * Test 1.5: xbar =/= 0, xp = 0, u =0, th1 + th2 > 0 
% * Test 1.6: xbar =/= 0, xp = 0, u =10, th1 + th2 > 0 
% * Test 1.7: xbar =/= 0, xp = 0, u =0, th1 <0, th2 = 0 
% * Test 1.8: xbar =/= 0, xp = 0, u =10, th1 <0, th2 = 0  

% Test 2
% xP = 1 (prosthetic swing)
% The following should hold in all cases:
%       xbar(k+1) = Abar*xbar(k)+Bubar*u+Baffbar 
%       (without any violated constraints)
% * Test 2.1: xbar = 0, xp = 1, u = 0 
% * Test 2.2: xbar = 0, xp = 1, u = -1
% * Test 2.3: xbar =/= 0, xp = 1, u =0, th1 + th2 < 0 
% * Test 2.4: xbar =/= 0, xp = 1, u =10, th1 + th2 < 0 
% * Test 2.5: xbar =/= 0, xp = 1, u =0, th1 + th2 > 0 
% * Test 2.6: xbar =/= 0, xp = 1, u =10, th1 + th2 > 0 
% * Test 2.7: xbar =/= 0, xp = 1, u =0, th1 <0, th2 = 0 
% * Test 2.8: xbar =/= 0, xp = 1, u =10, th1 <0, th2 = 0 

%% Build the 3 models around the 3 linearization points:
% Load the general model
dt = 1e-2;

%%% Model 1: x = 0
xeq1 = zeros(4,1);
% [Abar1, Bubar1, Baffbar1] = subsLinModel(xeq1, dt);
Abarc1 = Asymfun(xeq1,0);
Bubarc1 = Bsymfun(xeq1,0);
Baffbarc1 =  Baffsymfun(xeq1,0);
S1 = buildMLD(Abarc1, Bubarc1, Baffbarc1, dt);

%%% Model 2: x =/= 0, th1 + th2 < 0
xeq2 = [pi/10;-pi/8;1;1];
Abarc2 = Asymfun(xeq2,0);
Bubarc2 = Bsymfun(xeq2,0);
Baffbarc2 =  Baffsymfun(xeq2,0);
S2 = buildMLD(Abarc2, Bubarc2, Baffbarc2, dt);

%%% Model 3: x =/= 0, th1 + th2 > 0
xeq3 = [pi/8;-pi/10;1;1];
Abarc3 = Asymfun(xeq3,0);
Bubarc3 = Bsymfun(xeq3,0);
Baffbarc3 =  Baffsymfun(xeq3,0);
S3 = buildMLD(Abarc3, Bubarc3, Baffbarc3, dt);

%%% Model 4: x =/= 0, th1 <0,  th2 = 0
xeq4 = [-pi/8;0;1;1];
Abarc4 = Asymfun(xeq4,0);
Bubarc4 = Bsymfun(xeq4,0);
Baffbarc4 =  Baffsymfun(xeq4,0);
S4 = buildMLD(Abarc4, Bubarc4, Baffbarc4, dt);
%% Test 1: tests of xP = 0
% These tests are made to ensure x(k+1) = x(k) if xP = 0
xP = 0;

%%% xbar = 0 as linearization point
x0 = [xeq1;xP];

% Test 1.1: xbar = 0, xp = 0, u = 0
u = 0;
[x1, y0, flag] = onestepMLD(S1,x0,u);
if isequal(x1,x0) && ~flag
    disp('Test 1.1 successful')
else
    disp('Test 1.1: Error!') 
end

% Test 1.2: xbar = 0, xp = 0, u = -1
u = -1;
[x1, y0, flag] = onestepMLD(S1,x0,u);
if isequal(x1,x0) && ~flag
    disp('Test 1.2 successful')
else
    disp('Test 1.2: Error!') 
end

% Test 1.3: xbar =/= 0, xp = 0, u = 0, th1+th2 < 0
x0 = [xeq2;xP];
u = 0;
[x1, y0, flag] = onestepMLD(S2,x0,u);
if isequal(x1,x0) && ~flag
    disp('Test 1.3 successful')
else
    disp('Test 1.3: Error!') 
end

% Test 1.4: xbar =/= 0, xp = 0, u = -1, th1+th2 < 0
x0 = [xeq2;xP];
u = -1;
[x1, y0, flag] = onestepMLD(S2,x0,u);
if isequal(x1,x0) && ~flag
    disp('Test 1.4 successful')
else
    disp('Test 1.4: Error!') 
end

% Test 1.5: xbar =/= 0, xp = 0, u = 0, th1 + th2 > 0
x0 = [xeq3;xP];
u = 0;
[x1, y0, flag] = onestepMLD(S3,x0,u);
if isequal(x1,x0) && ~flag
    disp('Test 1.5 successful')
else
    disp('Test 1.5: Error!') 
end

% Test 1.6: xbar =/= 0, xp = 0, u = -1, th1 + th2 > 0
x0 = [xeq3;xP];
u = -1;
[x1, y0, flag] = onestepMLD(S3,x0,u);
if isequal(x1,x0) && ~flag
    disp('Test 1.6 successful')
else
    disp('Test 1.6: Error!') 
end

% Test 1.7: xbar =/= 0, xp = 0, u =0, th1 <0, th2 = 0 
x0 = [xeq4;xP];
u = 0;
[x1, y0, flag] = onestepMLD(S4,x0,u);
if isequal(x1,x0) && ~flag
    disp('Test 1.7 successful')
else
    disp('Test 1.7: Error!') 
end

% Test 1.8: xbar =/= 0, xp = 0, u =10, th1 <0, th2 = 0  
x0 = [xeq4;xP];
u = -1;
[x1, y0, flag] = onestepMLD(S4,x0,u);
if isequal(x1,x0) && ~flag
    disp('Test 1.8 successful')
else
    disp('Test 1.8: Error!') 
end

%% Test 2: tests of xP = 1
% These tests are made to ensure 
%       xbar(k+1) = Abar*xbar(k)+Bbar*u+Baffbar 
%       (without any violated constraints)
xP = 1;

%%% xbar = 0 as linearization point
x0 = [xeq1;xP];

% Test 2.1: xbar = 0, xp = 1, u = 0
u = 0;

x1comp = [(eye(4)+dt*Abarc1)*xeq1+dt*Bubarc1*u+dt*Baffbarc1; 1]; % Expect xP = 1
[x1, y0, flag] = onestepMLD(S1,x0,u);
if isequal(x1,x1comp) && ~flag
    disp('Test 2.1 successful')
elseif ~flag && norm(x1-x1comp) <= 2*eps
     disp('Test 2.1 successful, but there is an error <= 2*eps')  
else
    disp('Test 2.1: Error!') 
end


% Test 2.2: xbar = 0, xp = 1, u = -1
u = -1;

x1comp = [(eye(4)+dt*Abarc1)*xeq1+dt*Bubarc1*u+dt*Baffbarc1; 1]; % Expect xP = 1
[x1, y0, flag] = onestepMLD(S1,x0,u);
if isequal(x1,x1comp) && ~flag
    disp('Test 2.2 successful')
elseif ~flag && norm(x1-x1comp) <= 2*eps
     disp('Test 2.2 successful, but there is an error <= 2*eps')  
else
    disp('Test 2.2: Error!') 
end

% Test 2.3: xbar =/= 0, xp = 1, u = 0, th1+th2 < 0
x0 = [xeq2;xP];
u = 0;

x1comp = [(eye(4)+dt*Abarc2)*xeq2+dt*Bubarc2*u+dt*Baffbarc2; 1]; % Expect xP = 1
[x1, y0, flag] = onestepMLD(S2,x0,u);
if isequal(x1,x1comp) && ~flag
    disp('Test 2.3 successful')
elseif ~flag && norm(x1-x1comp) <= 2*eps
     disp('Test 2.3 successful, but there is an error <= 2*eps')  
else
    disp('Test 2.3: Error!') 
end

% Test 2.4: xbar =/= 0, xp = 1, u = -1, th1+th2 < 0
x0 = [xeq2;xP];
u = -1;

x1comp = [(eye(4)+dt*Abarc2)*xeq2+dt*Bubarc2*u+dt*Baffbarc2; 1]; % Expect xP = 1
[x1, y0, flag] = onestepMLD(S2,x0,u);
if isequal(x1,x1comp) && ~flag
    disp('Test 2.4 successful')
elseif ~flag && norm(x1-x1comp) <= 2*eps
     disp('Test 2.4 successful, but there is an error <= 2*eps')  
else
    disp('Test 2.4: Error!') 
end

% Test 2.5: xbar =/= 0, xp = 1, u = 0, th1 + th2 > 0
x0 = [xeq3;xP];
u = 0;

x1comp = [(eye(4)+dt*Abarc3)*xeq3+dt*Bubarc3*u+dt*Baffbarc3; 0]; % Expect xP = 0
[x1, y0, flag] = onestepMLD(S3,x0,u);
if isequal(x1,x1comp) && ~flag
    disp('Test 2.5 successful')
elseif ~flag && norm(x1-x1comp) <= 2*eps
     disp('Test 2.5 successful, but there is an error <= 2*eps')  
else
    disp('Test 2.5: Error!') 
end

% Test 2.6: xbar =/= 0, xp = 1, u = -1, th1 + th2 > 0
x0 = [xeq3;xP];
u = -1;

x1comp = [(eye(4)+dt*Abarc3)*xeq3+dt*Bubarc3*u+dt*Baffbarc3; 0]; % Expect xP = 0
[x1, y0, flag] = onestepMLD(S3,x0,u);
if isequal(x1,x1comp) && ~flag
    disp('Test 2.6 successful')
elseif ~flag && norm(x1-x1comp) <= 2*eps
     disp('Test 2.6 successful, but there is an error <= 2*eps')  
else
    disp('Test 2.6: Error!') 
end

% Test 2.7: xbar =/= 0, xp = 1, u =0, th1 <0, th2 = 0 
x0 = [xeq4;xP];
u = 0;

x1comp = [(eye(4)+dt*Abarc4)*xeq4+dt*Bubarc4*u+dt*Baffbarc4; 1]; % Expect xP = 1
[x1, y0, flag] = onestepMLD(S4,x0,u);

if isequal(x1,x1comp) && ~flag
    disp('Test 2.7 successful')
elseif ~flag && norm(x1-x1comp) <= 2*eps
     disp('Test 2.7 successful, but there is an error <= 2*eps')  
else
    disp('Test 2.7: Error!') 
end

% Test 2.8: xbar =/= 0, xp = 1, u =10, th1 <0, th2 = 0  
x0 = [xeq4;xP];
u = -1;

x1comp = [(eye(4)+dt*Abarc4)*xeq4+dt*Bubarc4*u+dt*Baffbarc4; 1]; % Expect xP = 1
[x1, y0, flag] = onestepMLD(S4,x0,u);
if isequal(x1,x1comp) && ~flag
    disp('Test 2.8 successful')
elseif ~flag && norm(x1-x1comp) <= 2*eps
     disp('Test 2.8 successful, but there is an error <= 2*eps')   
else
    disp('Test 2.8: Error!') 
    end