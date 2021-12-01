function [dStatedt] = ODEpmass(t,State,Param)

%unpack state
V=State(3);
thet=State(4);

%unpack param
m=Param.m;
alph=Param.alph;
g=Param.g;
b=Param.b;
rinv=Param.rinv(t);


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
