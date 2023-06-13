% TV quad Tracking Problem



Ts = 0.001;
t0 = 0;
x0 = [0,0,0,0,0,0,0,0.1047,0.2,1,0,0]';
t_sim = 0;
x_sim = x0.';
XD = x0.'
t = t0;
while t < 0.5
[t,x] = ode45(@(t,x) clsys(t,x),[t0 t0+Ts],x0);
t0 = t(end); 
x0 = x(end,:);
t_sim(end+1) = t(end);
x_sim(end+1,:) = x(end,:);

XD(:,end+1) = trajectory(t(end));
end

trajectory(0.01)

f1 = figure('Renderer', 'painters', 'Position', [10 10 1600 1000]);
hold on
title('States')
set(0, 'CurrentFigure', f1)
    for i = 1:12
    subplot(4,3,i)
    title("state",i)
    hold on 
    plot(t_sim(:),x_sim(:,i),'r')
    %plot(t(:),xd(:,i),'b')
    end

%% Functions

% closed - loop system
function x_dot = clsys(t,x,f1)
xd = trajectory(t);
[K_pos1,~,K_att1,~] = controllerParams;
[F,M] = fbl(x,xd,K_pos1,K_att1);
v = [F;M];
x_dot = plant(x,v);

% %set(0, 'CurrentFigure', f1)
% figure(1)
% axis('equal')
%     for i = 1:12
%     subplot(3,4,i)
%     hold on 
%     plot(t,x(i),'r')
%     plot(t,xd(i),'b')
%     end
end

% trajectory
function XD = trajectory(t)

tiltRate = pi/300;
altRate = 0.2;
circRadius = 1;
circRate = pi/30;

xd = circRadius*cos(circRate*t);
yd = circRadius*sin(circRate*t);
zd = altRate*t;
ud = -circRadius*circRate*sin(circRate*t);
vd = circRadius*circRate*cos(circRate*t);
wd = altRate;
pd = 0;
qd = 0;
rd = 0;
phid = 0;
thed = 0;
psid = 0;

XD = [pd;qd;rd;phid;thed;psid;ud;vd;wd;xd;yd;zd];
end
% controller 
function [F,M] = fbl(x,xd,K_pos1,K_att1)
% CALCULATE MOMENT AND THRUST FORCES
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

% plant 
function x_dot = plant(x,u)
%states 
%Position       
X = x(10);Y = x(11);Z = x(12);
%Velocity
U = x(7);V = x(8);W = x(9);
%Euler Angles
Phi = x(4);The = x(5);Psi = x(6);
%Body Rates
P = x(1);Q = x(2);R = x(3);

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
end