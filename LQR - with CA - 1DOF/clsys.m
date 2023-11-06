% closed - loop system
function x_dot = clsys(t,x)

xd = trajectory(t,x);
[K_pos1,K_pos2,K_att1,K_att2] = controllerParams;
[F,M] = lqr_controller(x,xd,K_pos1,K_pos2,K_att1,K_att2);
u_FM = [F;M];
u_R = controlAllocation(F,M);
u = rotorParams(u_R);
%u = CAplusFM(vd); % Dr. Reza's Code
v_actual = virtualControl(u,u_FM);

x_dot = plant(x,v_actual);
end