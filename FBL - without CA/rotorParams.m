function control_inputs = rotorParams(u_R)

L = [0.225, 0.225, 0];
kT = 2.98e-6;
kQ = 1.140e-7;

e1 = asin((u_R(1))/(sqrt(u_R(1)^2+u_R(2)^2+u_R(3)^2)));
e2 = asin((u_R(4))/(sqrt(u_R(4)^2+u_R(5)^2+u_R(6)^2)));
e3 = -asin((u_R(7))/(sqrt(u_R(7)^2+u_R(8)^2+u_R(9)^2)));
e4 = -asin((u_R(10))/(sqrt(u_R(10)^2+u_R(11)^2+u_R(12)^2)));

b1 = -asin((u_R(2))/(cos(e1)*sqrt(u_R(1)^2+u_R(2)^2+u_R(3)^2)));
b2 = -asin((u_R(5))/(cos(e2)*sqrt(u_R(4)^2+u_R(5)^2+u_R(6)^2)));
b3 = asin((u_R(8))/(cos(e3)*sqrt(u_R(7)^2+u_R(8)^2+u_R(9)^2)));
b4 = asin((u_R(11))/(cos(e4)*sqrt(u_R(10)^2+u_R(11)^2+u_R(12)^2)));

w1 = (30/pi)*sqrt(sqrt(u_R(1)^2+u_R(2)^2+u_R(3)^2)/(kT));
w2 = (30/pi)*sqrt(sqrt(u_R(4)^2+u_R(5)^2+u_R(6)^2)/(kT));
w3 = (30/pi)*sqrt(sqrt(u_R(7)^2+u_R(8)^2+u_R(9)^2)/(kT));
w4 = (30/pi)*sqrt(sqrt(u_R(10)^2+u_R(11)^2+u_R(12)^2)/(kT));

rotorSpeed = [w1,w2,w3,w4];
rotorRoll = [b1,b2,b3,b4];
rotorPitch = [e1,e2,e3,e4];

control_inputs = [w1,w2,w3,w4,b1,b2,b3,b4,e1,e2,e3,e4]';