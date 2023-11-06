function v_actual = virtualControl(u,u_FM)

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

T1= -k*(w(1)*(pi/30))^2;
T2= -k*(w(2)*(pi/30))^2;
T3= k*(w(3)*(pi/30))^2;
T4= k*(w(4)*(pi/30))^2;

T = [T1,T2,T3,T4]; % Rotor Thrust

%rotor forces
F_R = zeros(3,4);
for i =1:4
    F_R(:,i) = [-T(i)*sin(eta(i));
                +T(i)*cos(eta(i))*sin(beta(i));
                +T(i)*cos(beta(i))*cos(eta(i))];
end

%transform rotor forces to body frame
F_B = zeros(3,4);
F_B(:,1) = F_R(:,1);
F_B(:,2) = [-1,0,0;0,-1,0;0,0,1]*F_R(:,2);
F_B(:,3) = [1,0,0;0,-1,0;0,0,-1]*F_R(:,3);
F_B(:,4) = [-1,0,0;0,1,0;0,0,-1]*F_R(:,4);

%body forces 
FBx = F_B(1,1)+F_B(1,2)+F_B(1,3)+F_B(1,4);
FBy = F_B(2,1)+F_B(2,2)+F_B(2,3)+F_B(2,4);
FBz = F_B(3,1)+F_B(3,2)+F_B(3,3)+F_B(3,4);

MPhi = l(2)*(F_B(3,1)-F_B(3,2)-F_B(3,3)+F_B(3,4))...
    + l(3)*(F_B(2,1)+F_B(2,2)+F_B(2,3)+F_B(2,4));
MThe = l(1)*(-F_B(3,1)+F_B(3,2)-F_B(3,3)+F_B(3,4))...
    + l(3)*(-F_B(2,1)-F_B(2,2)-F_B(2,3)-F_B(2,4));
MPsi = (kQ)*(F_B(3,1)+F_B(3,2)-F_B(3,3)-F_B(3,4))...
    + l(1)*(F_B(2,1)-F_B(2,2)+F_B(2,3)-F_B(2,4))...
    + l(2)*(-F_B(1,1)+F_B(1,2)+F_B(1,3)-F_B(1,4));



v_actual = [FBx;FBy;FBz;-MPhi;-MThe;MPsi]; 

% v_actual1 = [sign(u_FM(1))*abs(FBx);sign(u_FM(2))*abs(FBy);...
%     sign(u_FM(3))*abs(FBz);sign(u_FM(4))*abs(MPhi);...
%     sign(u_FM(5))*abs(MThe);sign(u_FM(6))*abs(MPsi)]

end