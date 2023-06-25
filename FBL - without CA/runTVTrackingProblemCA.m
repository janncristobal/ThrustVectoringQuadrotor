clc;
clear;
%close all;
% TV quad Tracking Problem



% Ts = 0.001;
% t0 = 0;
% x0 = [0,0,0,0,0,0,0,0.1047,0.2,1,0,0]';
% t_sim = 0;
% x_sim = x0.';
% XD = x0.';
% t = t0;
% while t < 0.5
% [t,x] = ode45(@(t,x) clsys(t,x),[t0 t0+Ts],x0);
% t0 = t(end); 
% x0 = x(end,:);
% t_sim(end+1) = t(end);
% x_sim(end+1,:) = x(end,:);
% 
% XD(end+1,:) = trajectory(t(end)).';
% end
% 
% %trajectory(0.01)
% 
% f1 = figure('Renderer', 'painters', 'Position', [10 10 1600 1000]);
% hold on
% title('States')
% set(0, 'CurrentFigure', f1)
%     for i = 1:12
%     subplot(4,3,i)
%     title("state",i)
%     hold on 
%     plot(t_sim(:),x_sim(:,i),'r.-')
%     plot(t_sim(:),XD(:,i),'b')
%     end
% see if github dekstop can see this change
% yes it does -> problems are kinda fixed

%% Method 2
t0 = 0;
tf = 2;
%x0 = [0,0,0,0,0,0,0,0.1047,0.2,1,0,0]';
x0 = [0,0,0,0,0,0,0,0,0,0,0,0]';
[t,x] = ode45(@(t,x) clsys(t,x),[t0 tf],x0);

for ii = 1:length(t)
XD(ii,:) = trajectory(t(ii),x(ii,:)).';
end
%%
f1 = figure('Renderer', 'painters', 'Position', [10 10 1600 1000]);
hold on
title('States')
set(0, 'CurrentFigure', f1)
    for i = 1:12
        plotTitle = titlePlot(i);
    subplot(4,3,i)
    title(plotTitle)
    hold on 
    plot(t(1:10:end),x(1:10:end,i),'r.-')
    plot(t(1:10:end),XD(1:10:end,i),'b')
    end

%% Functions



% closed - loop system
function x_dot = clsys(t,x)
xd = trajectory(t,x);
[K_pos1,~,K_att1,~] = controllerParams;
[F,M] = fbl(x,xd,K_pos1,K_att1);
%v = [F;M];
u_R = controlAllocation(F,M);
u = rotorParams(u_R);
x_dot = plant2(x,u);

end

% trajectory
function XD = trajectory(t,x)

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

phid = xd;
thed = yd;
psid = zd;

% pd = phid-psid*sin(x(5));
% qd = thed*cos(x(4))+psid*sin(x(4))*cos(x(5));
% rd = -thed*sin(x(4))+psid*cos(x(4))*cos(x(5));

% pd = phid-psid*sin(thed);
% qd = thed*cos(phid)+psid*sin(phid)*cos(thed);
% rd = -thed*sin(phid)+psid*cos(phid)*cos(thed);

% pd = ud;
% qd = vd;
% rd = wd;

pd = ud-wd*sin(thed);
qd = vd*cos(phid)+wd*sin(phid)*cos(thed);
rd = -vd*sin(phid)+wd*cos(phid)*cos(thed);

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
% g_p = (1/m)*[cos(x(6))*cos(x(5)), cos(x(4))*sin(x(6)) + cos(x(6))*sin(x(4))*sin(x(5)), sin(x(4))*sin(x(6)) - cos(x(4))*cos(x(6))*sin(x(5)), 0, 0, 0;
% 					-cos(x(5))*sin(x(6)), cos(x(4))*cos(x(6)) - sin(x(4))*sin(x(6))*sin(x(5)), cos(x(6))*sin(x(4)) + cos(x(4))*sin(x(6))*sin(x(5)), 0, 0, 0;
% 					sin(x(5)), -cos(x(5))*sin(x(4)), cos(x(4))*cos(x(5)), 0, 0, 0;
% 					0,0,0,0,0,0;
% 					0,0,0,0,0,0;
% 					0,0,0,0,0,0];

% dU = (FBx*cos(Psi)*cos(The)...
%     + FBy*(cos(Phi)*sin(Psi) + cos(Psi)*sin(Phi)*sin(The))...
%     + FBz*(sin(Phi)*sin(Psi) - cos(Phi)*cos(Psi)*sin(The))...
%     - Ax*U)/m;
% dV = (-FBx*cos(The)*sin(Psi)...
%     + FBy*(cos(Phi)*cos(Psi) - sin(Phi)*sin(Psi)*sin(The))...
%     + FBz*(cos(Psi)*sin(Phi) + cos(Phi)*sin(Psi)*sin(The))...
%     - Ay*V)/m;
% dW = (g + (FBx*sin(The)...
%     - FBy*cos(The)*sin(Phi)...
%     + FBz*cos(Phi)*cos(The)...
%     - Az*W)/m);

Phi = x(4);
The = x(5);
Psi = x(6);
g_p = (1/m)*[cos(Psi)*cos(The), cos(The)*sin(Psi),-sin(The), 0, 0, 0;
					-cos(Phi)*sin(Psi) + sin(Phi)*sin(The)*cos(Psi), cos(Phi)*cos(Psi) + sin(Phi)*sin(The)*sin(Psi), sin(Phi)*cos(The), 0, 0, 0;
					sin(Phi)*sin(Psi) + cos(Phi)*sin(The)*cos(Psi), -sin(Phi)*cos(Psi) + cos(Phi)*sin(The)*sin(Psi), cos(Phi)*cos(The), 0, 0, 0;
					0,0,0,0,0,0;
					0,0,0,0,0,0;
					0,0,0,0,0,0];

% dU = (FBx*cos(Psi)*cos(The)...
%     + FBy*cos(The)*sin(Psi)...
%     - FBz*sin(The)...
%     - Ax*U)/m;
% dV = (FBx*(-cos(Phi)*sin(Psi) + sin(Phi)*sin(The)*cos(Psi))...
%     + FBy*(cos(Phi)*cos(Psi) + sin(Phi)*sin(The)*sin(Psi))...
%     + FBz*sin(Phi)*cos(The)...
%     - Ay*V)/m;
% dW = (g + (FBx*(sin(Phi)*sin(Psi) + cos(Phi)*sin(The)*cos(Psi))...
%     + FBy*(-sin(Phi)*cos(Psi) + cos(Phi)*sin(The)*sin(Psi))...
%     + FBz*cos(Phi)*cos(The)...
%     - Az*W)/m);


F = g_p(1:3,1:3)\(-(f_p(4:6).'*f_p(1:3))-100.*K_pos1*e_p);

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
M = g_a(1:3,1:3)\(-(f_a(4:6).'*f_a(1:3))-100.*K_att1*e_a);
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

% dU = (FBx*cos(Psi)*cos(The)...
%     + FBy*(cos(Phi)*sin(Psi) + cos(Psi)*sin(Phi)*sin(The))...
%     + FBz*(sin(Phi)*sin(Psi) - cos(Phi)*cos(Psi)*sin(The))...
%     - Ax*U)/m;
% dV = (-FBx*cos(The)*sin(Psi)...
%     + FBy*(cos(Phi)*cos(Psi) - sin(Phi)*sin(Psi)*sin(The))...
%     + FBz*(cos(Psi)*sin(Phi) + cos(Phi)*sin(Psi)*sin(The))...
%     - Ay*V)/m;
% dW = (g + (FBx*sin(The)...
%     - FBy*cos(The)*sin(Phi)...
%     + FBz*cos(Phi)*cos(The)...
%     - Az*W)/m);

dU = (FBx*cos(Psi)*cos(The)...
    + FBy*cos(The)*sin(Psi)...
    - FBz*sin(The)...
    - Ax*U)/m;
dV = (FBx*(-cos(Phi)*sin(Psi) + sin(Phi)*sin(The)*cos(Psi))...
    + FBy*(cos(Phi)*cos(Psi) + sin(Phi)*sin(The)*sin(Psi))...
    + FBz*sin(Phi)*cos(The)...
    - Ay*V)/m;
dW = (g + (FBx*(sin(Phi)*sin(Psi) + cos(Phi)*sin(The)*cos(Psi))...
    + FBy*(-sin(Phi)*cos(Psi) + cos(Phi)*sin(The)*sin(Psi))...
    + FBz*cos(Phi)*cos(The)...
    - Az*W)/m);

dX = U;
dY = V;
dZ = W;
x_dot = [dP; dQ; dR; dPhi; dThe; dPsi; dU; dV; dW; dX; dY; dZ];
end

function x_dot = plant2(x,u)

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

T1= -k*(w(1)*(pi/30))^2;
T2= -k*(w(1)*(pi/30))^2;
T3= k*(w(1)*(pi/30))^2;
T4= k*(w(1)*(pi/30))^2;

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


%body forces 
FBx = F_B(1,1)+F_B(1,2)+F_B(1,3)+F_B(1,4);
FBy = F_B(2,1)+F_B(2,2)+F_B(2,3)+F_B(2,4);
FBz = F_B(3,1)+F_B(3,2)+F_B(3,3)+F_B(3,4);



MPhi = l(2)*(F_B(3,1)+F_B(3,2)-F_B(3,3)-F_B(3,4))...
    + l(3)*(F_B(2,1)+F_B(2,2)+F_B(2,3)+F_B(2,4));
MThe = l(1)*(-F_B(3,1)+F_B(3,2)-F_B(3,3)+F_B(3,4))...
    + l(3)*(-F_B(2,1)-F_B(2,2)-F_B(2,3)-F_B(2,4));
MPsi = (kQ)*(F_B(3,1)+F_B(3,2)-F_B(3,3)-F_B(3,4))...
    + l(1)*(F_B(2,1)-F_B(2,2)+F_B(2,3)-F_B(2,4))...
    + l(2)*(-F_B(1,1)+F_B(1,2)+F_B(1,3)-F_B(1,4));

%Omega = -w(1)+w(2)-w(3)+w(4);

Omega = w(1)-w(2)+w(3)-w(4);

dP = (1/Ixx)*((Iyy-Izz)*Q*R+MPhi-Ir*Q*Omega-Ar*P);
dQ = (1/Iyy)*((Ixx-Izz)*P*R+MThe-Ir*P*Omega-Ar*Q);
dR = (1/Izz)*((Iyy-Ixx)*Q*P+MPsi-Ar*R);

dPhi= P + sin(Phi)*tan(The)*Q + cos(Phi)*tan(The)*R;
dThe= cos(Phi)*Q - sin(Phi)*R;
dPsi= sin(Phi)/cos(The)*Q + cos(Phi)/cos(The)*R;

dU = (FBx*cos(Psi)*cos(The)...
    + FBy*cos(The)*sin(Psi)...
    - FBz*sin(The)...
    - Ax*U)/m;
dV = (FBx*(-cos(Phi)*sin(Psi) + sin(Phi)*sin(The)*cos(Psi))...
    + FBy*(cos(Phi)*cos(Psi) + sin(Phi)*sin(The)*sin(Psi))...
    + FBz*sin(Phi)*cos(The)...
    - Ay*V)/m;
dW = (g + (FBx*(sin(Phi)*sin(Psi) + cos(Phi)*sin(The)*cos(Psi))...
    + FBy*(-sin(Phi)*cos(Psi) + cos(Phi)*sin(The)*sin(Psi))...
    + FBz*cos(Phi)*cos(The)...
    - Az*W)/m);

dX = U;
dY = V;
dZ = W;
x_dot = [dP; dQ; dR; dPhi; dThe; dPsi; dU; dV; dW; dX; dY; dZ];
end
% plot labels
function plotTitle = titlePlot(i)
    switch i
        case 1 
            plotTitle = "P";
        case 2 
            plotTitle = "Q";
        case 3 
            plotTitle = "R";
        case 4 
            plotTitle = "Phi";
        case 5 
            plotTitle = "Theta";
        case 6 
            plotTitle = "Psi";
        case 7 
            plotTitle = "U";
        case 8 
            plotTitle = "V";
        case 9 
            plotTitle = "W";
        case 10 
            plotTitle = "X";
        case 11
            plotTitle = "Y";
        case 12 
            plotTitle = "Z";
    end
end