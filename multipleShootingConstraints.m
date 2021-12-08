function [Cineq, Ceq] = multipleShootingConstraints(decisionVars,Param)
taugrid = reshape(decisionVars(1:Param.taugridNum*3),[],3);
tdur = decisionVars(Param.taugridNum*3+1);
slackVars = decisionVars(Param.taugridNum*3+2:end);
slackVars = reshape(slackVars,12,[]);
numShooting = size(slackVars,2);

L1=Param.L1;
L2=Param.L2;
L3=Param.L3;
L3cross=Param.L3cross;

Param.taugrid = taugrid;
options = odeset('abstol',1e-9,'reltol',1e-9);
numTGridCon = 100;
dt = tdur/numShooting;

stateList = [];
for i = 1:numShooting
    State0 = [slackVars(:,i);0];
    tgridCon = linspace((i-1)*dt,i*dt,numTGridCon);
    [~,shootiStates] = ode45(@ODEskateboard_wCost, tgridCon, State0, options, Param);
    finalState(:,i)=shootiStates(end,1:end-1)';
    stateList=[stateList; shootiStates(2:end,1:end-1)];
end

% unpack stateList

x=stateList(:,1);
y=stateList(:,2);
thetB=stateList(:,4);
thet1=stateList(:,5);
thet2=stateList(:,6);
thet3=stateList(:,7);

% not falling constraints
ry_knee=cos(thetB+thet1)*L1;% >0
ry_hip=ry_knee+cos(thetB+thet1-thet2)*L2;% >0
ry_torso=ry_hip+cos(thetB+thet1-thet2+thet3)*L3/2;
ry_front_torso=ry_torso - sin(thetB+thet1-thet2+thet3)*L3cross/2;% >0
ry_back_torso=ry_torso + sin(thetB+thet1-thet2+thet3)*L3cross/2;% >0
ry_head=ry_hip+cos(thetB+thet1-thet2+thet3)*L3; % >0

noFallCons= -[rl_knee; ry_hip; ry_front_torso; ry_back_torso; ry_head];

% range of motion constraints

xCon=[x-Param.xUB; Param.xLB-x];
yCon=[y-Param.yUB; Param.yLB-y];
thet1Con=[thet1-Param.thet1UB; Param.thet1LB-thet1];
thet2Con=[thet2-Param.thet2UB; Param.thet2LB-thet2];
thet3Con=[thet3-Param.thet3UB; Param.thet3LB-thet3];

RoMCons=[xCon; yCon; thet1Con; thet2Con; thet3Con];

% downhill travel constraints

dhCon= [y(end)-Param.yendUB; Param.yendLB-y(end)];

% torque limits

tauLims= [taugrid-Param.tauLim; -Param.tauLim-taugrid];

% Continuity Constraints (slack variable cons)

slackVarCons=slackVars(:,2:end)-finalState(:,1:end-1); % =0
slackVarCons=reshape(slackVarCons,[],1);
slackVarCons=[salckVarCons; reshape(slackVars(1:2,1),[],1)];

% periodicity constraints

perioCons=(stateList(end,[1,3:end]) - stateList(1,[1,3:end]))'; % = 0

perioCons=[perioCons; (taugrid(1,:)-taugrid(end,:))'];

% downhill velocity control constraint

VelCon= (y(end)-y(1))/tdur - Param.angVel;

% form the return vars
Cineq = [noFallCons; dhCon; RoMCons; tauLims];
Ceq = [slackVarCons; perioCons; VelCon];

end 

