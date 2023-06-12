% Thrust Vectoring Quadrotor without Control Allocation

% main body 
x0 = zeros(12,1);
tspan = linspace(0,100,elements);
options= odeset('abstol',1e-9,'reltol',1e-9);
[t,x] = ode45(@(t,x) clsys(t,x), [0 100], x0, options);

% plot states
figure(1)
for i = 1:12
    subplot(4,3,i)
    hold on 
    plot(t,x(:,i))
end
%% Functions

% closed loop system 
function x_dot = clsys(t,x)
    % desired trajectory
    xd = trajDesired(t);
    % controller
    [K_pos1,K_att1] = controllerParameters;
    [F,M] = fbl(x,xd,K_pos1,K_att1);
    u = [F;M];
    % plant
    x_dot = plant_TV(x,u);
end

% desired trajectory
function xd = trajDesired(t)
%xd = [p,q,r,phi,the,psi,u,v,w,x,y,z]

    if t < 50
        xd = [0;0;0;0;0;0;...
            0.5*pi*cos(pi*t);-0.5*pi*sin(pi*t);0.4;...
            0.5*sin(pi*t);0.5*cos(pi*t);0.4*t];
    else
        xd = [0;0;0;0;0;...
            0.5*pi*cos(pi*t);-0.5*pi*sin(pi*t);-0.4;...
            0.5*sin(pi*t);0.5*cos(pi*t);-0.4*t];
    end
end

% controller gain calculator
function [K_pos1,K_att1] = controllerParameters
% Quadrotor Params
%m=1.0230; %total mass of drone
Ixx=9.5e-03;Iyy=9.5e-03;Izz=1.86e-02;
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

%pControllability = rank(ctrb(ss(AP,BP,CP,DP)));

QP = CP'*CP;
RP = eye(3);

%Pos_d = [0;0;0;0;0;0];%x,y,z,u,v,w
K_pos1 = lqr(AP,BP,QP,RP);

% Attitude Controller (phi,the,psi,p,q,r)
AA = zeros(6,6);
AA(1,4) = 1;AA(2,5) = 1;AA(3,6) = 1;
AA(4,4) = -Ar/Ixx;AA(5,5) = -Ar/Iyy;AA(6,6) = -Ar/Izz;
BA = zeros(6,3);BA(4,1) = 1/Ixx;BA(5,2) = 1/Iyy;BA(6,3) = 1/Izz;
CA = zeros(3,6);
CA(1,1) = 1;CA(2,2) = 1;CA(3,3) = 1;

QA = CA'*CA;
RA = eye(3);

%Att_d = [0;0;0;0;0;0];
K_att1 = lqr(AA,BA,QA,RA);
end


% feedback linearization controller
function [F,M] = fbl(x,xd,K_pos1,K_att1)

%find m,Ixx,Iyy,Izz,Ir
m=0.468; 
%m = 0.68;
g = 9.81;
Ixx=4.856e-03;Iyy=4.856e-03;Izz=8.801e-03;

Ir=3.357e-05;
Ax=.3; Ay=0.3; Az=0.25; Ar=0.2;

Omega =0;

% states: x = [p,q,r,phi,the,psi,u,v,w,x,y,z];

% position control
% state error 
e_p = x(7:12) - xd(7:12);

f_p = [-(1/m)*(Ax*x(7));
    -(1/m)*(Ay*x(8));
    -(1/m)*(Az*x(9));
    x(7);
    x(8);
    x(9)];
g_p = (1/m)*[cos(x(6))*cos(x(5)), cos(x(4))*sin(x(6)) + cos(x(6))*sin(x(4))*sin(x(5)), sin(x(4))*sin(x(6)) - cos(x(4))*cos(x(6))*sin(x(5)), 0, 0, 0;
					-cos(x(5))*sin(x(6)), cos(x(4))*cos(x(6)) - sin(x(4))*sin(x(6))*sin(x(5)), cos(x(6))*sin(x(4)) + cos(x(4))*sin(x(6))*sin(x(5)), 0, 0, 0;
					sin(x(5)), -cos(x(5))*sin(x(4)), cos(x(4))*cos(x(5)), 0, 0, 0;
					0,0,0,0,0,0;
					0,0,0,0,0,0;
					0,0,0,0,0,0];
F = g_p(1:3,1:3)\(-(f_p(4:6).'*f_p(1:3))-K_pos1*e_p);

% attitude control 
e_a = x(1:6) - xd(1:6);
f_a = [(1/Ixx)*((Iyy-Izz)*x(2)*x(3)-Ir*x(2)*Omega-Ar*x(1));
    (1/Iyy)*((Ixx-Izz)*x(1)*x(3)-Ir*x(1)*Omega-Ar*x(2));
    (1/Izz)*((Iyy-Ixx)*x(2)*x(1)-Ar*x(3));
    x(1) + sin(x(4))*tan(x(5))*x(2) + cos(x(4))*tan(x(5))*x(3);
    cos(x(4))*x(2) - sin(x(4))*x(3);
    sin(x(4))/cos(x(5))*x(2) + cos(x(4))/cos(x(5))*x(3)];
g_a = [(1/Ixx),0,0,0,0,0;
    0,(1/Iyy),0,0,0,0;
    0,0,(1/Izz),0,0,0;
    0,0,0,0,0,0;
    0,0,0,0,0,0;
    0,0,0,0,0,0];

M = g_a(1:3,1:3)\(-(f_a(4:6).'*f_a(1:3))-K_att1*e_a);
end

% quadrotor dynamics (Plant)
function x_dot = plant_TV(x,u)
% states: x = [p,q,r,phi,the,psi,u,v,w,x,y,z]
P =x(1);Q = x(2);R = x(3); 
Phi = x(4); The = x(5); Psi = x(6);
U = x(7); V = x(8); W = x(9); 
X = x(10); Y = x(11); Z = x(12); 
% inputs: u = [Fx,Fy,Fz,Mx,My,Mz]
FBx = u(1);FBy =u(2);FBz =u(3);
MPhi = u(4); MThe = u(5); MPsi = u(6);
% quadparams: 
    %find m,Ixx,Iyy,Izz,Ir
    m=0.468; 
    %m = 0.68;
    g = 9.81;
    
    Ixx=4.856e-03;Iyy=4.856e-03;Izz=8.801e-03;
    
    Ir=3.357e-05;
    Ax=.3; Ay=0.3; Az=0.25; Ar=0.2;

    Omega =0;
%Differential Equations
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
end