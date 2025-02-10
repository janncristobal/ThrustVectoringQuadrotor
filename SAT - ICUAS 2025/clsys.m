% closed - loop system
function x_dot = clsys(t,x,controller,t_last)
global v_des v_act rotor_Params rotor_Forces
xd = trajectory1(t);
[K_pos1,K_pos2,K_att1,K_att2] = controllerParams;
[K_pos1aug,K_pos2aug,K_att1aug,K_att2aug, Ki_pos, Ki_att] = controllerParamsAug;
if controller == 1
    [F,M] = fbl(x,xd,K_pos1,K_att1);
elseif controller == 2
    [F,M] = lqr_control(x,xd,K_pos1,K_pos2,K_att1,K_att2);
elseif controller == 3
    dt = abs(t-t_last);
    [F,M] = lqr_aug(x, xd, K_pos1aug, K_pos2aug, K_att1aug, K_att2aug, Ki_pos, Ki_att,dt);
else 
    [F,M] = fbl(x,xd,K_pos1,K_att1);
end
u_FM = [F;M];
u_R = controlAllocation(F,M);
u = actuatorMixing(u_R);
%u = CAplusFM(vd); % Dr. Reza's Code
v_actual = virtualControl(u);

rotor_Forces = [rotor_Forces, u_R];
rotor_Params = [rotor_Params, u];
v_des = [v_des,u_FM]; % Force and Moment required based on the controller 
v_act = [v_act,v_actual]; % Force and Moment calculated based on the actuator inputs


% rotor_Params = [rotor_Params,u];
% rotor_Forces = [rotor_Forces,u_R];

x_dot = plant(x,v_actual);
end