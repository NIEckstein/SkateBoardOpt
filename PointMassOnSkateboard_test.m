clc;close all;clear;


tspan=linspace(0,100,10000);
Param.m=1;
Param.rinv=@(t) 0.5*cos(2*pi()*t);
Param.b=0;
Param.alph=pi()/3;
Param.g=9.81;
options=odeset('abstol',1e-9,'reltol',1e-9);
[tList,sList]=ode45(@ODEpmass,tspan,[0;0;0;0],options,Param);

plot(sList(:,1),sList(:,2));
axis equal