



% not falling constraint
ry_knee=cos(thetB+thet1)*L1;% >0
ry_hip=ry_knee+cos(thetB+thet1-thet2)*L2;% >0
ry_torso=ry_hip+cos(thetB+thet1-thet2+thet3)*L3/2;
ry_front_torso=ry_torso - sin(thetB+thet1-thet2+thet3)*L3cross/2;% >0
ry_back_torso=ry_torso + sin(thetB+thet1-thet2+thet3)*L3cross/2;% >0
ry_head=ry_hip+cos(thetB+thet1-thet2+thet3)*L3; % >0


% range of motion constraints
y; % [0 -50] meters
x; % [-12 12] meters
thet1; % [-pi/4 pi/4]
thet2; % [0 11*pi/12]
thet3; % [-pi/4   pi-atan(L3cross/L3)]

% periodicity constraints 

State(end,[1,3:end]) - State(1,[1,3:end]); % = 0

% average downhill velocity constraint

(State(end,2)-State(1,2))/tspan(end) - ydot_desired; % =0











