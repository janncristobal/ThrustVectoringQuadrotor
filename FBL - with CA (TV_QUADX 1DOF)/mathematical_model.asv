%Test all the rotation 
beta = [10;0;10;0];
eta = [0;10;0;10];
zeta = [315;225;45;135];
F_R = zeros(3,4);
Thrust = [1;1;1;1];
frd_ned = zeros(3,3,4);
ned_frd = zeros(3,3,4);
for i = 1:4
    frd_ned(:,:,i) = rotationMatrix(beta(i),eta(i),zeta(i));
    ned_frd(:,:,i) = transpose(rotationMatrix(beta(i),eta(i),zeta(i)));
    F_R(:,i) = frd_ned(:,:,i)*[0;0;Thrust(i)];
end




 

function R = rotationMatrix(angleX, angleY, angleZ)
    % Convert angles to radians
    thetaX = deg2rad(angleX);
    thetaY = deg2rad(angleY);
    thetaZ = deg2rad(angleZ);

    % Calculate rotation matrices about x, y, z axes
    Rx = [1, 0, 0; 0, cos(thetaX), -sin(thetaX); 0, sin(thetaX), cos(thetaX)];
    Ry = [cos(thetaY), 0, sin(thetaY); 0, 1, 0; -sin(thetaY), 0, cos(thetaY)];
    Rz = [cos(thetaZ), -sin(thetaZ), 0; sin(thetaZ), cos(thetaZ), 0; 0, 0, 1];

    % Combine the rotation matrices
    R = Rz * Ry * Rx;
end