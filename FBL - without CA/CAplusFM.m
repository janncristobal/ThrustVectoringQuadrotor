function v_actual = CAplusFM(vd)

lx = 0.225;
ly = 0.225;
lz = 0;
l = [lx,ly,lz];
k=2.98e-06;
m=0.468; 
g = 9.81;
Ixx=4.856e-03;Iyy=4.856e-03;Izz=8.801e-03;
Ir=3.357e-05;
Ax=0.3; Ay=0.3; Az=0.25; Ar=0.2;
L = [0.225, 0.225, 0];
kT = 2.98e-6;
kQ = 1.140e-7;


B = zeros(6,12);
% Fx
B(1,1) = 1; 
B(1,4) = 1;
B(1,7) = 1; 
B(1,10) =1; 
% Fy 
B(2,2) = 1; 
B(2,5) = 1;
B(2,8) = 1; 
B(2,11) =1; 
%Fz 
B(3,3) = -1; 
B(3,6) = -1;
B(3,9) = -1; 
B(3,12) = -1; 
%Mx
B(4,3) = L(2);
B(4,6) = -L(2);
B(4,9) = -L(2);
B(4,12) = L(2);
B(4,2) = L(3);
B(4,5) = L(3);
B(4,8) = L(3);
B(4,11) = L(3);
%My
B(5,3) = -L(1);
B(5,6) = L(1);
B(5,9) = -L(1);
B(5,12) = L(1);
B(5,1) = -L(3);
B(5,4) = -L(3);
B(5,7) = -L(3);
B(5,10) = -L(3);
%Mz
B(6,2) = L(1);
B(6,5) = -L(1);
B(6,8) = L(1);
B(6,11) = -L(1);
B(6,1) = -L(2);
B(6,4) = L(2);
B(6,7) = L(2);
B(6,10) = -L(2);
B(6,3) = kQ;
B(6,6) = kQ;
B(6,9) = -kQ;
B(6,12) = -kQ;

%vd = [0 0 -10 0 0 4]';

F_body = pinv(B)*vd;


C_R1_B = diag([ 1, 1, 1]);
C_R2_B = diag([-1,-1, 1]);
C_R3_B = diag([ 1,-1,-1]);
C_R4_B = diag([-1, 1,-1]);

F_rotor1 = C_R1_B * F_body(1:3);
F_rotor2 = C_R2_B * F_body(4:6);
F_rotor3 = C_R3_B * F_body(7:9);
F_rotor4 = C_R4_B * F_body(10:12);

e1 =  asin((F_rotor1(1))/norm(F_rotor1));
e2 =  asin((F_rotor2(1))/norm(F_rotor2));
e3 = -asin((F_rotor3(1))/norm(F_rotor3));
e4 = -asin((F_rotor4(1))/norm(F_rotor4));
b1 = -asin((F_rotor1(2))/(cos(e1)*norm(F_rotor1)));
b2 = -asin((F_rotor2(2))/(cos(e2)*norm(F_rotor2)));
b3 =  asin((F_rotor3(2))/(cos(e3)*norm(F_rotor3)));
b4 =  asin((F_rotor4(2))/(cos(e4)*norm(F_rotor4)));

w1 = (30/pi)*sqrt(norm(F_rotor1)/(kT));
w2 = (30/pi)*sqrt(norm(F_rotor2)/(kT));
w3 = (30/pi)*sqrt(norm(F_rotor3)/(kT));
w4 = (30/pi)*sqrt(norm(F_rotor4)/(kT));

w    = [w1,w2,w3,w4];
beta = [b1,b2,b3,b4];
eta  = [e1,e2,e3,e4];

%% Conversion begins


nT1 = k*(w(1)*pi/30)^2;
nT2 = k*(w(2)*pi/30)^2;
nT3 = k*(w(3)*pi/30)^2;
nT4 = k*(w(4)*pi/30)^2;

nF_rotor1 = [nT1*sin(e1); -nT1*cos(e1)*sin(b1); -nT1*cos(b1)*cos(e1)];
nF_rotor2 = [nT2*sin(e2); -nT1*cos(e2)*sin(b2); -nT1*cos(b2)*cos(e2)];
nF_rotor3 = [-nT3*sin(e3); nT1*cos(e3)*sin(b3); nT1*cos(b3)*cos(e3)];
nF_rotor4 = [-nT4*sin(e4); nT1*cos(e4)*sin(b4); nT1*cos(b4)*cos(e4)];

nF_body(:,1) = C_R1_B' * nF_rotor1;
nF_body(:,2) = C_R2_B' * nF_rotor2;
nF_body(:,3) = C_R3_B' * nF_rotor3;
nF_body(:,4) = C_R4_B' * nF_rotor4;

nF_body1 = [nF_body(:,1);nF_body(:,2);nF_body(:,3);nF_body(:,4)];

%body forces 
nFBx = nF_body(1,1)+nF_body(1,2)+nF_body(1,3)+nF_body(1,4);
nFBy = nF_body(2,1)+nF_body(2,2)+nF_body(2,3)+nF_body(2,4);
nFBz = nF_body(3,1)+nF_body(3,2)+nF_body(3,3)+nF_body(3,4);

nMPhi = l(2)*(nF_body(3,1)+nF_body(3,2)-nF_body(3,3)-nF_body(3,4))...
    + l(3)*(nF_body(2,1)+nF_body(2,2)+nF_body(2,3)+nF_body(2,4));

nMThe = l(1)*(-nF_body(3,1)+nF_body(3,2)-nF_body(3,3)+nF_body(3,4))...
    + l(3)*(-nF_body(2,1)-nF_body(2,2)-nF_body(2,3)-nF_body(2,4));

nMPsi = (kQ)*(nF_body(3,1)+nF_body(3,2)-nF_body(3,3)-nF_body(3,4))...
    + l(1)*(nF_body(2,1)-nF_body(2,2)+nF_body(2,3)-nF_body(2,4))...
    + l(2)*(-nF_body(1,1)+nF_body(1,2)+nF_body(1,3)-nF_body(1,4));

v_actual = [nFBx;nFBy;nFBz;nMPhi;nMThe;nMPsi];

%[F_body nF_body1]

end