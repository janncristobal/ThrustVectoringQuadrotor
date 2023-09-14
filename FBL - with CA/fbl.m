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

Phi = x(4);
The = x(5);
Psi = x(6);
g_p = (1/m)*[cos(Psi)*cos(The), cos(The)*sin(Psi),-sin(The), 0, 0, 0;
					-cos(Phi)*sin(Psi) + sin(Phi)*sin(The)*cos(Psi), cos(Phi)*cos(Psi) + sin(Phi)*sin(The)*sin(Psi), sin(Phi)*cos(The), 0, 0, 0;
					sin(Phi)*sin(Psi) + cos(Phi)*sin(The)*cos(Psi), -sin(Phi)*cos(Psi) + cos(Phi)*sin(The)*sin(Psi), cos(Phi)*cos(The), 0, 0, 0;
					0,0,0,0,0,0;
					0,0,0,0,0,0;
					0,0,0,0,0,0];

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
M = g_a(1:3,1:3)\(-(f_a(4:6).'*f_a(1:3))-125.*K_att1*e_a);
end