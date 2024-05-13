function control_inputs = rotorParams(u_R)

L = [0.225, 0.225, 0];
kT = 2.98e-6;
kQ = 1.140e-7;

T = [sqrt(u_R(1)^2+u_R(2)^2+u_R(3)^2);
    sqrt(u_R(4)^2+u_R(5)^2+u_R(6)^2);
    sqrt(u_R(7)^2+u_R(8)^2+u_R(9)^2);
    sqrt(u_R(10)^2+u_R(11)^2+u_R(12)^2)];

e = zeros(4,1);
b = zeros(4,1);
w = zeros(4,1);
for i = 1:4
   e(i) = -asin(u_R(i*3-2)/T(i));%[rad]
   b(i) = asin(u_R(i*3-1)/(cos(e(i))*T(i))); %[rad]
   w(i) = (30/pi)*sqrt(T(i)/kT); %[rpm]
end 

control_inputs = [w(1:4);b(1:4);e(1:4)];