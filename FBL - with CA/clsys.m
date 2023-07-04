% closed - loop system
function x_dot = clsys(t,x)
global v_des v_act
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

% figure(2)
% hold on 
% subplot(3,2,1)
% hold on
% plot(t,u_FM(1));
% plot(t,v_actual(1));
% subplot(3,2,2)
% hold on
% plot(t,u_FM(2));
% plot(t,v_actual(2));
% subplot(3,2,3)
% hold on
% plot(t,u_FM(3));
% plot(t,v_actual(3));
% subplot(3,2,4)
% hold on
% plot(t,u_FM(4));
% plot(t,v_actual(4));
% subplot(3,2,5)
% hold on
% plot(t,u_FM(5));
% plot(t,v_actual(5));
% subplot(3,2,6)
% hold on
% plot(t,u_FM(6));
% plot(t,v_actual(6));

x_dot = plant(x,v_actual);
end