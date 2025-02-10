function [K_pos1,K_pos2,K_att1,K_att2, Ki_pos, Ki_att] = controllerParamsAug
% Quadrotor Params
Ixx=9.5e-03;Iyy=9.5e-03;Izz=1.86e-02;
m=0.468; 
g = 9.81;
Ir=3.357e-05;
Ax=.3; Ay=0.3; Az=0.25; Ar=0.2;

%% Position Controller (x,y,z,u,v,w) 
AP = zeros(6,6);
AP(1,4) = 1; AP(2,5) = 1; AP(3,6) = 1;
AP(4,4) = -Ax/m; AP(5,5) = -Ay/m; AP(6,6) = -Az/m;

BP = zeros(6,3);
BP(4,1) = 1/m; BP(5,2) = 1/m; BP(6,3) = 1/m;

CP = zeros(3,6);
CP(1,1) = 1; CP(2,2) = 1; CP(3,3) = 1;

% Augment Position System (Adding Integral Action)
AP_aug = [AP, zeros(6,3); -CP, zeros(3,3)];
BP_aug = [BP; zeros(3,3)];
CP_aug = [CP, zeros(3,3)];

QP = blkdiag(CP'*CP, eye(3));  % Augmented Q
RP = eye(3);

% Compute LQR Gain
K_aug_pos = lqr(AP_aug, BP_aug, QP, RP);

% Extract State Feedback and Integral Gains
K_pos1 = K_aug_pos(:,1:6);  % Position Feedback
Ki_pos = K_aug_pos(:,7:9);  % Integral Gain
K_pos2 = inv(BP'*BP) * BP' * AP;  % Feedforward

%% Attitude Controller (phi, the, psi, p, q, r)
AA = zeros(6,6);
AA(1,4) = 1; AA(2,5) = 1; AA(3,6) = 1;
AA(4,4) = -Ar/Ixx; AA(5,5) = -Ar/Iyy; AA(6,6) = -Ar/Izz;

BA = zeros(6,3);
BA(4,1) = 1/Ixx; BA(5,2) = 1/Iyy; BA(6,3) = 1/Izz;

CA = zeros(3,6);
CA(1,1) = 1; CA(2,2) = 1; CA(3,3) = 1;

% Augment Attitude System (Adding Integral Action)
AA_aug = [AA, zeros(6,3); -CA, zeros(3,3)];
BA_aug = [BA; zeros(3,3)];
CA_aug = [CA, zeros(3,3)];

QA = blkdiag(CA'*CA, eye(3));  % Augmented Q
RA = eye(3);

% Compute LQR Gain
K_aug_att = lqr(AA_aug, BA_aug, QA, RA);

% Extract State Feedback and Integral Gains
K_att1 = K_aug_att(:,1:6);  % Attitude Feedback
Ki_att = K_aug_att(:,7:9);  % Integral Gain
K_att2 = inv(BA'*BA) * BA' * AA;  % Feedforward

% disp('Position Gains:');
% disp(K_pos1);
% disp('Integral Position Gain:');
% disp(Ki_pos);
% 
% disp('Attitude Gains:');
% disp(K_att1);
% disp('Integral Attitude Gain:');
% disp(Ki_att);
% Feb 10
end
