function [dStatedt] = ODEmasswinertias(t,State,Param)

%unpack state
xdot=State(2);
ydot=State(4);
thetD=State(5);
thetDdot=State(6);
thetB=State(7);
thetBdot=State(8);

%unpack param
m=Param.m;
alph=Param.alph;
g=Param.g;
C=Param.C;
thetBdotdot=Param.thetBdotdot(t);


% %interpolate the torque grid
% tau=interp1(tgrid,Param.taugrid,t);
% tau1=tau(1);
% tau2=tau(2);

% apply equations of motion 
xdotdot=-(thetDdot*cos(thetD)*ydot^2*tan(thetD)^2 + thetDdot*cos(thetD)*ydot^2 + g*xdot*sin(alph)*cos(thetD))/(ydot*cos(thetD) - xdot*sin(thetD));
ydotdot=-(ydot*(thetDdot*ydot*sin(thetD)*tan(thetD)^2 + thetDdot*ydot*sin(thetD) + g*sin(alph)*cos(thetD)))/(ydot*cos(thetD) - xdot*sin(thetD));
thetDdotdot=-(C*thetBdot*xdot*sin(thetD)*(xdot^2 + ydot^2)^(1/2) - C*thetBdot*ydot*cos(thetD)*(xdot^2 + ydot^2)^(1/2) + (C*thetB*thetDdot*ydot^3*sin(thetD))/(xdot^2 + ydot^2)^(1/2) + (C*thetB*thetDdot*ydot^3*sin(thetD)*tan(thetD)^2)/(xdot^2 + ydot^2)^(1/2) + (C*thetB*thetDdot*xdot*ydot^2*cos(thetD))/(xdot^2 + ydot^2)^(1/2) + (C*g*thetB*xdot^2*sin(alph)*cos(thetD))/(xdot^2 + ydot^2)^(1/2) + (C*g*thetB*ydot^2*sin(alph)*cos(thetD))/(xdot^2 + ydot^2)^(1/2) + (C*thetB*thetDdot*xdot*ydot^2*cos(thetD)*tan(thetD)^2)/(xdot^2 + ydot^2)^(1/2))/(ydot*cos(thetD) - xdot*sin(thetD));
 


% form the time derivative of the state vector
dStatedt=[xdot;xdotdot;ydot;ydotdot;thetDdot;thetDdotdot;thetBdot;thetBdotdot];

end