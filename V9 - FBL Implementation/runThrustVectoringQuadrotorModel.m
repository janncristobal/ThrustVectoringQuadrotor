% script for initializing parameters and running the Model

clc;clear;close all;
addpath("qcat\");

%% Quadrotor Parameters
% Moment of Inertia
Ixx=9.5e-03;Iyy=9.5e-03;Izz=1.86e-02;
I = [Ixx 0 0;0 Iyy 0;0 0 Izz]; % Inertia Tensor
 
% Mass and acceleration due to gravity
m = 0.468;
g = 9.81;

% rotor constants 
kT = 2.98e-6;
kQ = 1.14e-7; 

Ir = 3.357e-5;
Ax = 0.3;Ay = 0.3;Az = 0.25; Ar = 0.2; 

%% Controller Design 
[K_pos1,K_pos2,K_att1,K_att2] = controllerParams;

%% Trajectory Planning 
elements = 1001;
simTime = linspace(0,100,elements);
%XD = trajectory1(simTime,elements);
XD = trajectory3(simTime,elements);
pth.p_d = timeseries(XD(1,:),simTime);
pth.q_d = timeseries(XD(2,:),simTime);
pth.r_d = timeseries(XD(3,:),simTime);
pth.phi_d = timeseries(XD(4,:),simTime);
pth.the_d = timeseries(XD(5,:),simTime);
pth.psi_d = timeseries(XD(6,:),simTime);
pth.u_d = timeseries(XD(7,:),simTime);
pth.v_d = timeseries(XD(8,:),simTime);
pth.w_d = timeseries(XD(9,:),simTime);
pth.x_d = timeseries(XD(10,:),simTime);
pth.y_d = timeseries(XD(11,:),simTime);
pth.z_d = timeseries(XD(12,:),simTime);

%% Run Simulink

    % intial condition
    x0 = [0,0,0,0,0,0,0,0,0.2,0,0,0]'; 
    % simulation time
    stime = 100;
    % Run quadsim.slx Simulink Model.
    out = sim('simulinkTVModel.slx',stime);

    %% Drone Position and Attitude Animation
p_act(1,:) = out.quad_state(:,1);
q_act(1,:) = out.quad_state(:,2);
r_act(1,:) = out.quad_state(:,3);
phi_act(1,:) = out.quad_state(:,4);
the_act(1,:) = out.quad_state(:,5);
psi_act(1,:) = out.quad_state(:,6);
u_act(1,:) = out.quad_state(:,7);
v_act(1,:) = out.quad_state(:,8);
w_act(1,:) = out.quad_state(:,9);
x_act(1,:) = out.quad_state(:,10);
y_act(1,:) = out.quad_state(:,11);
z_act(1,:) = out.quad_state(:,12); 
t_act(1,:) = out.tout(:);

states_act = out.quad_state';
rotor_Params = out.quad_rotorParams';
rotor_Forces = out.quad_rotorForces';

TV_PosAtt_Animation(states_act,rotor_Params,rotor_Forces)

%drone_Animation(x_act,y_act,z_act,phi_act,the_act,psi_act);
%% Thrust Vectoring Drone Animation
%tv_Animation(out.quad_rotorParams);
