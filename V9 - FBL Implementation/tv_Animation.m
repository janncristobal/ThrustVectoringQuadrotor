function tv_Animation(rotorParams)
%{
Inputs: T1,T2,T3,T4,beta1,beta2,beta3,beta4,eta1,eta2,eta3,eta4
%}

% Initialize
elements = width(rotorParams');
kT = 2.98e-06;
for i = 1:4
    omega(i,:) = rotorParams(:,i);
    beta(i,:) = 30.*rotorParams(:,i+4);
    eta(i,:) = 30.*rotorParams(:,i+8);
    
    if i < 3
    T(i,:) = -kT.*omega(i,:).^2;
    else
    T(i,:) = kT.*omega(i,:).^2;
    end
    T_Rx(i,:) = -T(i,:).*sin(eta(i,:));
    T_Ry(i,:) = T(i,:).*cos(eta(i,:)).*sin(beta(i,:));
    T_Rz(i,:) = T(i,:).*cos(eta(i,:)).*cos(beta(i,:));
end

% Rotor 1
    T_Bx(1,:) = T_Rx(1,:);
    T_By(1,:) = T_Ry(1,:);
    T_Bz(1,:) = T_Rz(1,:);
% Rotor 2
    T_Bx(2,:) = -T_Rx(2,:);
    T_By(2,:) = -T_Ry(2,:);
    T_Bz(2,:) = T_Rz(2,:);
% Rotor 3
    T_Bx(3,:) = T_Rx(3,:);
    T_By(3,:) = -T_Ry(3,:);
    T_Bz(3,:) = -T_Rz(3,:);
% Rotor 4
    T_Bx(4,:) = -T_Rx(4,:);
    T_By(4,:) = T_Ry(4,:);
    T_Bz(4,:) = -T_Rz(4,:);
% Sum of Forces
    T_Bx(5,:) = T_Bx(1,:)+T_Bx(2,:)+T_Bx(3,:)+T_Bx(4,:);
    T_By(5,:) = T_By(1,:)+T_By(2,:)+T_By(3,:)+T_By(4,:);
    T_Bz(5,:) = T_Bz(1,:)+T_Bz(2,:)+T_Bz(3,:)+T_Bz(4,:);
%tspan = linspace(0,10,100);

% %set thrust, beta, and eta as a function of time. 
% T1 = linspace(100,100,100); T2 = linspace(100,100,100); 
% T3 = linspace(100,100,100); T4 = linspace(100,100,100);
% b1 = linspace(pi/8,-pi/8,100); b2 = linspace(pi/8,-pi/8,100);
% b3 = linspace(pi/8,-pi/8,100); b4 = linspace(pi/8,-pi/8,100);
% e1 = linspace(pi/8,-pi/8,100); e2 = linspace(-pi/8,pi/8,100); 
% e3 = linspace(-pi/8,pi/8,100); e4 = linspace(pi/8,-pi/8,100);
% % e1 = linspace(0,0,100); e2 = linspace(0,0,100); 
% % e3 = linspace(0,0,100); e4 = linspace(0,0,100);

% note that the row corresponds to rotor 1,2,3,4 and the column is the
% value at each interval. (4x100)
%T = [T1;T2;T3;T4]; %thrust [N]
%beta = [b1;b2;b3;b4]; % roll [rad] 
%eta = [e1;e2;e3;e4]; % pitch [rad]



% for i = 1:100:elements
% F1R(:,i) = T(1,i).*[sin(eta(1,i));-cos(eta(1,i))*sin(beta(1,i));...
%     -cos(beta(1,i))*cos(eta(1,i))];
% F2R(:,i) = T(2,i).*[sin(eta(2,i));-cos(eta(2,i))*sin(beta(2,i));...
%     -cos(beta(2,i))*cos(eta(2,i))];
% F3R(:,i) = T(3,i).*[-sin(eta(3,i));cos(eta(3,i))*sin(beta(3,i));...
%     cos(beta(3,i))*cos(eta(3,i))];
% F4R(:,i) = T(4,i).*[-sin(eta(4,i));cos(eta(4,i))*sin(beta(4,i));...
%     cos(beta(4,i))*cos(eta(4,i))];
% F1B(:,i) =  F1R(:,i);
% F2B(:,i) =  [-1,0,0;0,-1,0;0,0,1]*F2R(:,i);
% F3B(:,i) =  [1,0,0;0,-1,0;0,0,-1]*F3R(:,i);
% F4B(:,i) =  [-1,0,0;0,1,0;0,0,-1]*F4R(:,i);
% FB_sum(:,i) = F1B(:,i)+F2B(:,i)+F3B(:,i)+F4B(:,i);
% end

r1_pos = [2;2;0];
r2_pos = [-2;-2;0];
r3_pos = [2;-2;0];
r4_pos = [-2;2;0];

X_body = [0,1;0,0;0,0;0,0;0,0];
Y_body = [0,0;0,0;0,1;0,0;0,0];
Z_body = [0,0;0,0;0,0;0,0;0,1];

F_scale = 0.5;
F_scale2 = 150;

figure 
view(53,15.5);
    for i = 1:100:elements
    clf
    %body axes
    plot3(X_body,Y_body,Z_body,'-b');
    hold on
    plot3([r1_pos(1) (r1_pos(1)+T_Bx(1,i)/F_scale)],...
        [r1_pos(2) (r1_pos(2)+T_By(1,i)/F_scale)],...
        [r1_pos(3) (r1_pos(3)+T_Bz(1,i)/F_scale)],'r')
    hold on
    plot3([r2_pos(1) (r2_pos(1)+T_Bx(2,i)/F_scale)],...
        [r2_pos(2) (r2_pos(2)+T_By(2,i)/F_scale)],...
        [r2_pos(3) (r2_pos(3)+T_Bz(2,i)/F_scale)],'g')
    hold on
    plot3([r3_pos(1) (r3_pos(1)+T_Bx(3,i)/F_scale)],...
        [r3_pos(2) (r3_pos(2)+T_By(3,i)/F_scale)],...
        [r3_pos(3) (r3_pos(3)+T_Bz(3,i)/F_scale)],'b')
    hold on
    plot3([r4_pos(1) (r4_pos(1)+T_Bx(4,i)/F_scale)],...
        [r4_pos(2) (r4_pos(2)+T_By(4,i)/F_scale)],...
        [r4_pos(3) (r4_pos(3)+T_Bz(4,i)/F_scale)],'k')
    hold on
    plot3([0 (T_Bx(5,i)/F_scale2)],...
        [0 (T_By(5,i)/F_scale2)],...
        [0 (T_Bz(5,i)/F_scale2)])
    hold on

    xlim([-5 5])
    ylim([-5 5])
    zlim([-5 5])
    view([1 1 -1])
    xlabel('x')
    ylabel('y')
    zlabel('z')
    drawnow;
    grid on;
    view()
    pause(0.01)
    end

end