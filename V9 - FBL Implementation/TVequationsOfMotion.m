% TV EquationOfMotion

function x_dot = TVequationsOfMotion(x,u)

%states 
%Position       
X = x(10);Y = x(11);Z = x(12);
%Velocity
U = x(7);V = x(8);W = x(9);
%Euler Angles
Phi = x(4);The = x(5);Psi = x(6);
%Body Rates
P = x(1);Q = x(2);R = x(3);

%inputs 
%Rotor Speed 
w = [u(1),u(2),u(3),u(4)];
% %Rotor Roll 
% beta = [-u(5),-u(6),u(7),u(8)];
% %Rotor Pitch 
% eta = [-u(9),-u(10),u(11),u(12)];

%Rotor Roll 
beta = [u(5),u(6),u(7),u(8)];
%Rotor Pitch 
eta = [u(9),u(10),u(11),u(12)];

% Quad Dimensions
lx = 0.225;
ly = 0.225;
lz = 0;
l = [lx,ly,lz]; % length from the center of mass

% CALCULATE MOMENT AND THRUST FORCES
%find k,d,l
k=2.98e-06; kQ = 1.14e-7;%kQ=.0382; 
%find m,Ixx,Iyy,Izz,Ir
m=0.468; 
%m = 0.68;
g = 9.81;

Ixx=4.856e-03;Iyy=4.856e-03;Izz=8.801e-03;

Ir=3.357e-05;
Ax=.3; Ay=0.3; Az=0.25; Ar=0.2;
% T1= -k*w(1)^2;
% T2= -k*w(2)^2;
% T3= k*w(3)^2;
% T4= k*w(4)^2;

T1= -k*w(1)^2;
T2= -k*w(2)^2;
T3= k*w(3)^2;
T4= k*w(4)^2;

T = [T1,T2,T3,T4]; % Rotor Thrust

%rotor forces
for i =1:4
    F_R(:,i) = [-T(i)*sin(eta(i));
                +T(i)*cos(eta(i))*sin(beta(i));
                +T(i)*cos(beta(i))*cos(eta(i))];
end

%transform rotor forces to body frame
F_B(:,1) = F_R(:,1);
F_B(:,2) = [-1,0,0;0,-1,0;0,0,1]*F_R(:,2);
F_B(:,3) = [1,0,0;0,-1,0;0,0,-1]*F_R(:,3);
F_B(:,4) = [-1,0,0;0,1,0;0,0,-1]*F_R(:,4);

% F_B(:,1) = F_R(:,1);
% F_B(:,2) = F_R(:,2);
% F_B(:,3) = F_R(:,3);
% F_B(:,4) = F_R(:,4);

%body forces 
FBx = F_B(1,1)+F_B(1,2)+F_B(1,3)+F_B(1,4);
FBy = F_B(2,1)+F_B(2,2)+F_B(2,3)+F_B(2,4);
FBz = F_B(3,1)+F_B(3,2)+F_B(3,3)+F_B(3,4);

% MPhi = l(2)*(-F_R(3,1)+F_R(3,2)+F_R(3,3)-F_R(3,4));
% MThe = l(1)*(F_R(3,1)-F_R(3,2)+F_R(3,3)-F_R(3,4));
% MPsi = kQ*(-F_R(3,1)+F_R(3,2)-F_R(3,3)+F_R(3,4));

MPhi = l(2)*(F_B(3,1)+F_B(3,2)-F_B(3,3)-F_B(3,4))...
    + l(3)*(F_B(2,1)+F_B(2,2)+F_B(2,3)+F_B(2,4));
MThe = l(1)*(-F_B(3,1)+F_B(3,2)-F_B(3,3)+F_B(3,4))...
    + l(3)*(-F_B(2,1)-F_B(2,2)-F_B(2,3)-F_B(2,4));
% MPsi = kQ*(-F_B(3,1)+F_B(3,2)-F_B(3,3)+F_B(3,4))...
%     + l(1)*(F_B(2,1)-F_B(2,2)+F_B(2,3)-F_B(2,4))...
%     + l(2)*(-F_B(1,1)-F_B(1,2)+F_B(1,3)+F_B(1,4));
MPsi = (kQ)*(F_B(3,1)+F_B(3,2)-F_B(3,3)-F_B(3,4))...
    + l(1)*(F_B(2,1)-F_B(2,2)+F_B(2,3)-F_B(2,4))...
    + l(2)*(-F_B(1,1)+F_B(1,2)+F_B(1,3)-F_B(1,4));
%MPsi = kQ*(F_B(3,1)+F_B(3,2)-F_B(3,3)-F_B(3,4));

%Omega = -w(1)+w(2)-w(3)+w(4);

Omega = w(1)-w(2)+w(3)-w(4);

dP = (1/Ixx)*((Iyy-Izz)*Q*R+MPhi-Ir*Q*Omega-Ar*P);
dQ = (1/Iyy)*((Ixx-Izz)*P*R+MThe-Ir*P*Omega-Ar*Q);
dR = (1/Izz)*((Iyy-Ixx)*Q*P+MPsi-Ar*R);

% dP = (1/Ixx)*((Izz-Iyy)*Q*R+MPhi-Ir*Q*Omega-Ar*P);
% dQ = (1/Iyy)*((Ixx-Izz)*P*R+MThe+Ir*P*Omega-Ar*Q);
% dR = (1/Izz)*((Iyy-Ixx)*Q*P+MPsi-Ar*R);

dPhi= P + sin(Phi)*tan(The)*Q + cos(Phi)*tan(The)*R;
dThe= cos(Phi)*Q - sin(Phi)*R;
dPsi= sin(Phi)/cos(The)*Q + cos(Phi)/cos(The)*R;

dU = (FBx*cos(Psi)*cos(The)...
    + FBy*(cos(Phi)*sin(Psi) + cos(Psi)*sin(Phi)*sin(The))...
    + FBz*(sin(Phi)*sin(Psi) - cos(Phi)*cos(Psi)*sin(The))...
    - Ax*U)/m;
dV = (-FBx*cos(The)*sin(Psi)...
    + FBy*(cos(Phi)*cos(Psi) - sin(Phi)*sin(Psi)*sin(The))...
    + FBz*(cos(Psi)*sin(Phi) + cos(Phi)*sin(Psi)*sin(The))...
    - Ay*V)/m;
dW = (g + (FBx*sin(The)...
    - FBy*cos(The)*sin(Phi)...
    + FBz*cos(Phi)*cos(The)...
    - Az*W)/m);

dX = U;
dY = V;
dZ = W;
x_dot = [dP; dQ; dR; dPhi; dThe; dPsi; dU; dV; dW; dX; dY; dZ];