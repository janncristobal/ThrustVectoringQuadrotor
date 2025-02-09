% closed - loop system
function x_dot = clsys(t,x)
global v_des v_act rotor_w_b_e
xd = trajectory(t,x);
[K_pos1,~,K_att1,~] = controllerParams;
[F,M] = fbl(x,xd,K_pos1,K_att1);
u_FM = [F;M];
u_R = controlAllocation(F,M);
u = rotorParams(u_R);
%u = CAplusFM(vd); % Dr. Reza's Code
v_actual = virtualControl(u,u_FM);

v_des = [v_des,u_FM];
v_act = [v_act,v_actual];
rotor_w_b_e = [rotor_w_b_e,u_R];


x_dot = plant(x,v_actual);
end