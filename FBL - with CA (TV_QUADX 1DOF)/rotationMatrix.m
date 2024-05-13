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