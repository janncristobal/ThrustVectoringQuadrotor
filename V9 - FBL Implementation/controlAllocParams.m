function [B,umin,umax] = controlAllocParams
L = [0.225, 0.225, 0];
 kT = 2.98e-6;
 kQ = 1.140e-7;

% v = [F;M]; %virtual work  
B = zeros(6,12);
%Fx
B(1,1) = 1; B(1,4) = 1; B(1,7) = 1; B(1,10) =1; 
% Fy 
B(2,2) = 1; B(2,5) = 1; B(2,8) = 1; B(2,11) =1; 
%Fz 
B(3,3) = -1; B(3,6) = -1; B(3,9) = -1; B(3,12) = -1; 
%Mx
B(4,3) = L(2); B(4,6) = -L(2); B(4,9) = -L(2); B(4,12) = L(2);
B(4,2) = L(3); B(4,5) = L(3); B(4,8) = L(3); B(4,11) = L(3);
%My
B(5,3) = -L(1); B(5,6) = L(1); B(5,9) = -L(1); B(5,12) = L(1);
B(5,1) = -L(3); B(5,4) = -L(3); B(5,7) = -L(3); B(5,10) = -L(3);
%Mz
B(6,2) = L(1); B(6,5) = -L(1); B(6,8) = L(1); B(6,11) = -L(1);
B(6,1) = -L(2); B(6,4) = L(2); B(6,7) = L(2); B(6,10) = -L(2);
B(6,3) = kQ; B(6,6) = kQ; B(6,9) = -kQ; B(6,12) = -kQ;

umin = -1000.*ones(12,1);
umax = 1000.*ones(12,1);

plim = 1000.*[-1,1;-1,1;-1,1;-1,1;-1,1;-1,1;-1,1;-1,1;-1,1;-1,1;-1,1;-1,1];
rlim = 1000.*[-1,1;-1,1;-1,1;-1,1;-1,1;-1,1;-1,1;-1,1;-1,1;-1,1;-1,1;-1,1];
end