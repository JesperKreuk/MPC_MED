function Bsym = Bsymfun(in1,uCMG)
%BSYMFUN
%    BSYM = BSYMFUN(IN1,UCMG)

%    This function was generated by the Symbolic Math Toolbox version 8.3.
%    29-Jul-2021 21:07:56

th1 = in1(1,:);
th2 = in1(2,:);
t2 = -th2;
t3 = t2+th1;
t4 = cos(t3);
t5 = t4.^2;
t6 = t5.*6.474885125266016e+30;
t7 = t6-3.577692360622564e+31;
t8 = 1.0./t7;
Bsym = [0.0;0.0;t4.*t8.*-2.165692500986667e+30;t8.*-3.712328178393856e+31];
