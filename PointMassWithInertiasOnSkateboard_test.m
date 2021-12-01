clc;close all;clear;


tspan=linspace(0,10,1000);
Param.m=1;
Param.thetBdotdot=@(t) 0;
Param.C=1;
Param.alph=pi()/3;
Param.g=9.81;
options=odeset('abstol',1e-9,'reltol',1e-9);
[tList,sList]=ode45(@ODEmasswinertias,tspan,[0;1;0;0;.1;0;-.1;0],options,Param);

plot(sList(:,1),sList(:,3));
axis equal