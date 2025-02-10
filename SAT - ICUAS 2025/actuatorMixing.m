function control_inputs = actuatorMixing(u)
%u = [f_lon_1, f_lat_1,f_ver_1,f_lon_2, f_lat_2,f_ver_2,f_lon_3, f_lat_3,f_ver_3,f_lon_4, f_lat_4,f_ver_4]
kT = 2.98e-6;

b = zeros(4,1); % roll, angle about longitudinal axis
e = zeros(4,1); % pitch, angle about lateral axis
w = zeros(4,1); % rotorSpeed, rpm
T = zeros(4,1); % Total Thrust
f_R = zeros(3,4);
f_B = zeros(3,4);
%u_R =  
theta = 0; 
b_min = -pi/4; 
b_max = pi/4;
e_min = -pi/4; 
e_max = pi/4;
for i = 1:4
    f_B(:,i) = [u(i*3-2);u(i*3-1);u(i*3)]; 
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
    %f_B(:,i) = transpose(R_rot2bod)*f_B(:,i);
    T(i,1) = norm(f_B(:,i));
    e(i,1) = -asin(f_B(1,i)/T(i,1));% should be the negative
    b(i,1) = atan2(f_B(2,i),f_B(3,i));% should be the negative
    w(i,1) = (30/pi)*sqrt(T(i,1)/kT);
    %saturation limits
    e(i,1) = min(max(e(i,1), e_min), e_max);
    b(i,1) = min(max(b(i,1), b_min), b_max);
end

control_inputs = [w;b;e];
end