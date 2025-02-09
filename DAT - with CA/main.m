% Run this script to start the simulation 

clc; 
clear; 
addpath("qcat")

% Initial Conditions 
t0 = 0; 
tf = 100; 
x0 = zeros(12,1);

%Solve the differential equation 
[t,x] = ode45(@(t,x) clsys(t,x), [t0,tf], x0);


% clsys.m is the closed-loop system 
    % trajectory.m -> xd = trajectory(t,x);
    % contParams.m -> [k_pos1,k_pos2,k_att1,k_att2] = contParams
    % fbl.m -> [F,M] = fbl(x,xd,k_pos1,k_att1);
    % contAlloc.m -> u_R = contAlloc(F,M); 
        % u_R = 12x1 xyz forces by each rotor
    % actMixing.m -> u = actMixing(u_R)
        % u = 12x1 rotorSpeed, beta, etad
    % virCont.m
    % plant.m 