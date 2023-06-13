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

% Quad Dimensions
lx = 0.225;
ly = 0.225;
lz = 0;

% CALCULATE MOMENT AND THRUST FORCES
%find k,d,l
% k=2.98e-06; kQ = 1.14e-7;%kQ=.0382; 
%find m,Ixx,Iyy,Izz,Ir
m=0.468; 
%m = 0.68;
g = 9.81;

Ixx=4.856e-03;Iyy=4.856e-03;Izz=8.801e-03;

Ir=3.357e-05;
Ax=.3; Ay=0.3; Az=0.25; Ar=0.2;


%body forces 
FBx = u(1);
FBy = u(2);
FBz = u(3);

MPhi = u(4);
MThe = u(5);
MPsi = u(6);


Omega = 0;

dP = (1/Ixx)*((Iyy-Izz)*Q*R+MPhi-Ir*Q*Omega-Ar*P);
dQ = (1/Iyy)*((Ixx-Izz)*P*R+MThe-Ir*P*Omega-Ar*Q);
dR = (1/Izz)*((Iyy-Ixx)*Q*P+MPsi-Ar*R);

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