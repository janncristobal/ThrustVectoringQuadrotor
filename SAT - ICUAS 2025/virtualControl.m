function v_actual = virtualControl(u)

%inputs 
%Rotor Speed 
w = [u(1),u(2),u(3),u(4)];
% %Rotor Roll 
% beta = [-u(5),-u(6),u(7),u(8)];
% %Rotor Pitch 
% eta = [-u(9),-u(10),u(11),u(12)];

%Rotor Roll 
beta = 1.*[u(5),u(6),u(7),u(8)];
%Rotor Pitch 
eta = 1.*[u(9),u(10),u(11),u(12)];

% Quad Dimensions
lx = 0.225;
ly = 0.225;
lz = 0;
%l = [lx,ly,lz]; % length from the center of mass
l = 0.29; 
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


%rotor forces
T = zeros(4,1);
F_R = zeros(3,4);
F_B = zeros(3,4);
for i =1:4
    if i == 1
        theta = deg2rad(315);
    elseif i == 2             
        theta = deg2rad(135);
    elseif i == 3
        theta = deg2rad(45);
    elseif i == 4
        theta = deg2rad(225);
    end
    R_rot2bod = [cos(theta),-sin(theta),0;
        sin(theta),cos(theta),0;
        0,0,1];
    T(i) = k*(w(i)*(pi/30))^2;
    F_R(:,i) = [-T(i)*sin(eta(i));
                +T(i)*cos(eta(i))*sin(beta(i));
                +T(i)*cos(beta(i))*cos(eta(i))];
    F_B(:,i) = R_rot2bod*F_R(:,i);

end

%transform rotor forces to body frame
% F_B = zeros(3,4);
% F_B(:,1) = F_R(:,1);
% F_B(:,2) = [-1,0,0;0,-1,0;0,0,1]*F_R(:,2);
% F_B(:,3) = [1,0,0;0,-1,0;0,0,-1]*F_R(:,3);
% F_B(:,4) = [-1,0,0;0,1,0;0,0,-1]*F_R(:,4);

%body forces 
FBx = F_B(1,1)+F_B(1,2)+F_B(1,3)+F_B(1,4);
FBy = F_B(2,1)+F_B(2,2)+F_B(2,3)+F_B(2,4);
FBz = F_B(3,1)+F_B(3,2)+F_B(3,3)+F_B(3,4);

% MPhi = l(2)*(F_B(3,1)-F_B(3,2)-F_B(3,3)+F_B(3,4))...
%     + l(3)*(F_B(2,1)+F_B(2,2)+F_B(2,3)+F_B(2,4));
% MThe = l(1)*(-F_B(3,1)+F_B(3,2)-F_B(3,3)+F_B(3,4))...
%     + l(3)*(-F_B(2,1)-F_B(2,2)-F_B(2,3)-F_B(2,4));
% MPsi = (kQ)*(F_B(3,1)+F_B(3,2)-F_B(3,3)-F_B(3,4))...
%     + l(1)*(F_B(2,1)-F_B(2,2)+F_B(2,3)-F_B(2,4))...
%     + l(2)*(-F_B(1,1)+F_B(1,2)+F_B(1,3)-F_B(1,4));
k_S = sqrt(2)/2;
MPhi = k_S*l*(-F_R(3,1)+F_R(3,2)+F_R(3,3)-F_R(3,4));
MThe = k_S*l*(-F_R(3,1)+F_R(3,2)-F_R(3,3)+F_R(3,4));
MPsi = l*(F_R(2,1)+F_R(2,2)+F_R(2,3)+F_R(2,4))+...
    kQ*(-F_R(3,1)-F_R(3,2)+F_R(3,3)+F_R(3,4));
v_actual = [FBx;FBy;FBz;MPhi;MThe;MPsi]; 

% v_actual1 = [sign(u_FM(1))*abs(FBx);sign(u_FM(2))*abs(FBy);...
%     sign(u_FM(3))*abs(FBz);sign(u_FM(4))*abs(MPhi);...
%     sign(u_FM(5))*abs(MThe);sign(u_FM(6))*abs(MPsi)]
% Feb 10
end