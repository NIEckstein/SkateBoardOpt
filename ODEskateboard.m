function [dStatedt] = ODEskateboard(t,State,Param)

%unpack state
thetD=State(3);
thetB=State(4);
thet1=State(5);
thet2=State(6);
thet3=State(7);
V=State(8);
thetBdot=State(9);
thet1dot=State(10);
thet2dot=State(11);
thet3dot=State(12);
%unpack param
m=Param.m;
alph=Param.alph;
g=Param.g;
K=Param.K;
B=Param.B;
L1=Param.L1;
L2=Param.L2;
L3=Param.L3;
Ib=Param.Ib;


% %interpolate the torque grid
% tau=interp1(tgrid,Param.taugrid,t);
% tau1=tau(1);
% tau2=tau(2);

% apply equations of motion 
dVdt=g*sin(alph)*cos(thet)-b*V/m;
dthetdt=V*rinv;
dxdt=V*sin(thet);
dydt=-V*cos(thet);

% form the time derivative of the state vector
dStatedt=[dxdt;dydt;dVdt;dthetdt];

end
