%% This version measures the angles with respect to the body frame
function u = controlAllocation(F,M)

% F(1) = Fx ; F(2) = Fy ; F(3) = Fz
% M(1) = Fx ; M(2) = Fy ; M(3) = Mz
L = [0.225, 0.225, 0];
kT = 2.98e-6;
kQ = 1.140e-7;

v_ca = [M;F]; %virtual work

% control allocation parameters
[B_ca,umin_ca,umax_ca] = controlAllocParams;

%solve for u using pseudo inverse
u = pinv(B_ca)*v_ca;
% Sequential Least Square 
%u = sls_alloc(B_ca,v_ca,umin_ca,umax_ca); %-> not working 
% Minimal Least Square
%u = mls_alloc(B_ca,v_ca,umin_ca,umax_ca); %-> stops at 10 seconds
% Weighted Least Square
% Wv = eye(6);
% Wu = eye(12);
% u = wls_alloc(B_ca,v_ca,umin_ca,umax_ca,Wv,Wu); %-> works really well

%u_R = [u(1);u(2);u(3);-u(4);-u(5);u(6);u(7);-u(8);-u(9);-u(10);u(11);-u(12)];
% Feb 10
end