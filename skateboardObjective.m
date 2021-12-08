function [cost] = skateboardObjective(decisionVars,Param)
taugrid = reshape(decisionVars(1:Param.taugridNum*3),[],3);
tdur = decisionVars(Param.taugridNum*3+1);
slackVars = decisionVars(Param.taugridNum*3+2:end);
slackVars = reshape(slackVars,12,[]);
numShooting = size(slackVars,2);
    
   
    Param.taugrid = taugrid;
    options = odeset('abstol',1e-9,'reltol',1e-9);
 
    dt = tdur/numShooting;
    
    stateList = [];
    lastCost=0;
    for i = 1:numShooting
        State0 = [slackVars(:,i);lastCost];
        tspan = [(i-1)*dt  i*dt];
        [~,shootiStates] = ode45(@ODEskateboard_wCost, tspan, State0, options, Param);
        lastCost=shootiStates(end,end)+lastCost;
    end
cost=lastCost;
end