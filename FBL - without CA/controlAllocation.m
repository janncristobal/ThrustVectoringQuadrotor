%% This version measures the angles with respect to the body frame
function u_R = controlAllocation(F,M)

% F(1) = Fx ; F(2) = Fy ; F(3) = Fz
% M(1) = Fx ; M(2) = Fy ; M(3) = Mz
L = [0.225, 0.225, 0];
kT = 2.98e-6;
kQ = 1.140e-7;

v_ca = [F;M]; %virtual work

% control allocation parameters
[B_ca,umin_ca,umax_ca] = controlAllocParams;

%solve for u using pseudo inverse
u = pinv(B_ca)*v_ca;
% Sequential Least Square 
%u = sls_alloc(B_ca,v_ca,umin_ca,umax_ca); %-> not working 
% Minimal Least Square
%u = mls_alloc(B_ca,v_ca,umin_ca,umax_ca); %-> stops at 10 seconds
% Weighted Least Square
Wv = eye(6);
Wu = eye(12);
%Wu(1,1) = 1;Wu(2,2) = 2;Wu(3,3) = 3;
%Wu(4,4) = 4;Wu(5,5) = 5;Wu(6,6) = 6;
%Wu(7,7) = 7;Wu(8,8) = 8;Wu(9,9) = 9;
%Wu(10,10) = 0.1;Wu(11,11) = 0.1;Wu(12,12) = 0.1;
%u = wls_alloc(B_ca,v_ca,umin_ca,umax_ca,Wv,Wu); %-> works really well


% Interior Point Method
%u = ip_alloc(B_ca,v_ca,umin_ca,umax_ca); % -> diverging
% Fixed Point Iterations
%u = fxp_alloc(B_ca,v_ca,umin_ca,umax_ca); % -> large steady state errors

% Cascading Generalized Inverses
%u = cgi_alloc(B_ca,v_ca,umin_ca,umax_ca);

%u = [T1x,T1y,T1z,T2x,T2y,T2z,T3x,T3y,T3z,T4x,T4y,T4z]

u_R = [u(1);u(2);u(3);-u(4);-u(5);u(6);u(7);-u(8);-u(9);-u(10);u(11);-u(12)];


% e1 = asin((u_R(1))/(sqrt(u_R(1)^2+u_R(2)^2+u_R(3)^2)));
% e2 = asin((u_R(4))/(sqrt(u_R(4)^2+u_R(5)^2+u_R(6)^2)));
% e3 = -asin((u_R(7))/(sqrt(u_R(7)^2+u_R(8)^2+u_R(9)^2)));
% e4 = -asin((u_R(10))/(sqrt(u_R(10)^2+u_R(11)^2+u_R(12)^2)));
% 
% b1 = -asin((u_R(2))/(cos(e1)*sqrt(u_R(1)^2+u_R(2)^2+u_R(3)^2)));
% b2 = -asin((u_R(5))/(cos(e2)*sqrt(u_R(4)^2+u_R(5)^2+u_R(6)^2)));
% b3 = asin((u_R(8))/(cos(e3)*sqrt(u_R(7)^2+u_R(8)^2+u_R(9)^2)));
% b4 = asin((u_R(11))/(cos(e4)*sqrt(u_R(10)^2+u_R(11)^2+u_R(12)^2)));
% 
% w1 = (30/pi)*sqrt(sqrt(u(1)^2+u(2)^2+u(3)^2)/(kT));
% w2 = (30/pi)*sqrt(sqrt(u(4)^2+u(5)^2+u(6)^2)/(kT));
% w3 = (30/pi)*sqrt(sqrt(u(7)^2+u(8)^2+u(9)^2)/(kT));
% w4 = (30/pi)*sqrt(sqrt(u(10)^2+u(11)^2+u(12)^2)/(kT));
% 
% rotorSpeed = [w1,w2,w3,w4];
% rotorRoll = [b1,b2,b3,b4];
% rotorPitch = [e1,e2,e3,e4];
% 
% control_inputs = [w1,w2,w3,w4,b1,b2,b3,b4,e1,e2,e3,e4];