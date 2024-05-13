function v_actual = virtualControl(u)

%inputs 
%Rotor Speed 
w = [u(1),u(2),u(3),u(4)]; % rpm
%Rotor Roll 
beta = rad2deg([u(5),u(6),u(7),u(8)]);
%Rotor Pitch 
eta = rad2deg([u(9),u(10),u(11),u(12)]);
%z-axis rotation from frd to ned frame

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

T = zeros(4,1);
zeta = [315;225;45;135];
F_B = zeros(3,4);
frd_ned = zeros(3,3,4);
%frame transformation from frd to ned frame of reference
for i = 1:4
    T(i)= -k*(w(i)*(pi/30))^2;
    frd_ned(:,:,i) = rotationMatrix(beta(i),eta(i),zeta(i));
    F_B(:,i) = frd_ned(:,:,i)*[0;0;T(i)];
end

%body forces 
FBx = F_B(1,1)+F_B(1,2)+F_B(1,3)+F_B(1,4);
FBy = F_B(2,1)+F_B(2,2)+F_B(2,3)+F_B(2,4);
FBz = F_B(3,1)+F_B(3,2)+F_B(3,3)+F_B(3,4);

MPhi = l(2)*(F_B(3,1)-F_B(3,2)-F_B(3,3)+F_B(3,4))...
    + l(3)*(-F_B(2,1)-F_B(2,2)-F_B(2,3)-F_B(2,4));
MThe = l(1)*(-F_B(3,1)+F_B(3,2)-F_B(3,3)+F_B(3,4))...
    + l(3)*(F_B(2,1)+F_B(2,2)+F_B(2,3)+F_B(2,4));
MPsi = (kQ)*(-F_B(3,1)-F_B(3,2)+F_B(3,3)+F_B(3,4))...
    + l(1)*(F_B(2,1)-F_B(2,2)+F_B(2,3)-F_B(2,4))...
    + l(2)*(-F_B(1,1)+F_B(1,2)+F_B(1,3)-F_B(1,4));

v_actual = [FBx;FBy;FBz;MPhi;MThe;MPsi]; 
end