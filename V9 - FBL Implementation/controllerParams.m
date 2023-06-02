function [K_pos1,K_pos2,K_att1,K_att2] = controllerParams
% Quadrotor Params
%m=1.0230; %total mass of drone
Ixx=9.5e-03;Iyy=9.5e-03;Izz=1.86e-02;
%I = [Ixx 0 0;0 Iyy 0;0 0 Izz]; % Inertia Tensor

%find k,d,l
%k=2.98e-06; kQ = 1.14e-7;%kQ=.0382; 
%find m,Ixx,Iyy,Izz,Ir
m=0.468; 
%m = 0.68;
g = 9.81;

Ir=3.357e-05;
Ax=.3; Ay=0.3; Az=0.25; Ar=0.2;

% Postition Controller (x,y,z,u,v,w) 
AP = zeros(6,6);
AP(1,4) = 1;AP(2,5) = 1;AP(3,6) = 1;
AP(4,4) = -Ax/m;AP(5,5) = -Ay/m;AP(6,6) = -Az/m;
BP = zeros(6,3);BP(4,1) = 1/m;BP(5,2) = 1/m;BP(6,3) = 1/m;
CP = zeros(3,6);
CP(1,1) = 1;CP(2,2) = 1;CP(3,3) = 1;
%CP(1,4) = 1;CP(2,5) = 1;CP(3,6) = 1;
%DP = zeros(3,3);

%pControllability = rank(ctrb(ss(AP,BP,CP,DP)));

QP = CP'*CP;
%QP(4,4) = 10;QP(5,5) = 10;QP(6,6) = 10;
RP = eye(3);

%Pos_d = [0;0;0;0;0;0];%x,y,z,u,v,w
K_pos1 = lqr(AP,BP,QP,RP);
K_pos2 = inv(BP'*BP)*BP'*AP;

% Attitude Controller (phi,the,psi,p,q,r)
AA = zeros(6,6);
AA(1,4) = 1;AA(2,5) = 1;AA(3,6) = 1;
AA(4,4) = -Ar/Ixx;AA(5,5) = -Ar/Iyy;AA(6,6) = -Ar/Izz;
BA = zeros(6,3);BA(4,1) = 1/Ixx;BA(5,2) = 1/Iyy;BA(6,3) = 1/Izz;
CA = zeros(3,6);
CA(1,1) = 1;CA(2,2) = 1;CA(3,3) = 1;
%CA(1,4) = 1;CA(2,5) = 1;CA(3,6) = 1;
%DA = zeros(3,3);

%aControllability = rank(ctrb(ss(AA,BA,CA,DA)));

QA = CA'*CA;
%QA(4,4) = 10;QA(5,5) = 10;QA(6,6) = 10;
RA = eye(3);

%Att_d = [0;0;0;0;0;0];
K_att1 = lqr(AA,BA,QA,RA);
K_att2 = inv(BA'*BA)*BA'*AA;

eig(AP-BP*K_pos1);
eig(AA-BA*K_att1);
end