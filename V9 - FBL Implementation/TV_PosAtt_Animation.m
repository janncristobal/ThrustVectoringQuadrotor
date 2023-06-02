% TV Animation 

function TV_PosAtt_Animation(states,rotor_Params,rotor_Forces)

x = states(10,:);
y = states(11,:);
z = states(12,:);
roll = states(4,:);
pitch = states(5,:); 
yaw = states(6,:);

beta = rotor_Params(5:8,:); %rotor Roll
eta = rotor_Params(9:12,:); %rotor Pitch

rotor_f1 = rotor_Forces(1:3,:); %forces in the x,y,z
rotor_f2 = rotor_Forces(4:6,:); %forces in the x,y,z
rotor_f3 = rotor_Forces(7:9,:); %forces in the x,y,z
rotor_f4 = rotor_Forces(10:12,:); %forces in the x,y,z

x1 = 0.2;y1 = 0.2;z1 = 0;
x2 = -0.2;y2 = -0.2;z2 = 0;
x3 = -0.2;y3 = 0.2;z3 = 0;
x4 = 0.2;y4 = -0.2;z4 = 0;

% This Animation code is for QuadCopter. Written by Jitendra Singh 


%% Define design parameters
D2R = pi/180;
R2D = 180/pi;
b   = 0.6;   % the length of total square cover by whole body of quadcopter in meter
a   = b/3;   % the legth of small square base of quadcopter(b/4)
H   = 0.06;  % hight of drone in Z direction (4cm)
H_m = H+H/2; % hight of motor in z direction (5 cm)
r_p = b/4;   % radius of propeller
%% Conversions
ro = 45*D2R;                   % angle by which rotate the base of quadcopter
Ri = [cos(ro) -sin(ro) 0;
      sin(ro) cos(ro)  0;
       0       0       1];     % rotation matrix to rotate the coordinates of base 
base_co = [-a/2  a/2 a/2 -a/2; % Coordinates of Base 
           -a/2 -a/2 a/2 a/2;
             0    0   0   0];
base = Ri*base_co;             % rotate base Coordinates by 45 degree 

to = linspace(0, 2*pi);
xp = r_p*cos(to);
yp = r_p*sin(to);
zp = zeros(1,length(to));
%% Define Figure plot
 fig1 = figure('pos', [0 50 800 600]);
 hg   = gca;
 view(53,15.5);
 grid on;
 axis equal;

  xlim([-4 4]); ylim([-4 4]); zlim([0 3.5]);
% xlim([-1 1]); ylim([-1 1]); zlim([-0.5 1]);
%xlim([-2.5 2.5]); ylim([-2.5 2.5]); zlim([0 3.5]);
 %title('(JITENDRA) Drone Animation')
 %title('Thrust Vectoring Quadrotor Animation')
 xlabel('X[m]');
 ylabel('Y[m]');
 zlabel('Z[m]');
 hold(gca, 'on');
 
%% Design Different parts
% design the base square
 drone(1) = patch([base(1,:)],[base(2,:)],[base(3,:)],'r');
 drone(2) = patch([base(1,:)],[base(2,:)],[base(3,:)+H],'r');
 alpha(drone(1:2),0.7);
% design 2 parpendiculer legs of quadcopter 
 [xcylinder,ycylinder,zcylinder] = cylinder([H/2 H/2]);
 drone(3) =  surface(b*zcylinder-b/2,ycylinder,xcylinder+H/2,'facecolor','b');
 drone(4) =  surface(ycylinder,b*zcylinder-b/2,xcylinder+H/2,'facecolor','b') ; 
 alpha(drone(3:4),0.6);
% % design 4 cylindrical motors 
%  drone(5) = surface(xcylinder+b/2,ycylinder,H_m*zcylinder+H/2,'facecolor','r');
%  drone(6) = surface(xcylinder-b/2,ycylinder,H_m*zcylinder+H/2,'facecolor','r');
%  drone(7) = surface(xcylinder,ycylinder+b/2,H_m*zcylinder+H/2,'facecolor','r');
%  drone(8) = surface(xcylinder,ycylinder-b/2,H_m*zcylinder+H/2,'facecolor','r');
%  alpha(drone(5:8),0.7);
% % design 4 propellers
%  drone(9)  = patch(xp+b/2,yp,zp+(H_m+H/2),'c','LineWidth',0.5);
%  drone(10) = patch(xp-b/2,yp,zp+(H_m+H/2),'c','LineWidth',0.5);
%  drone(11) = patch(xp,yp+b/2,zp+(H_m+H/2),'p','LineWidth',0.5);
%  drone(12) = patch(xp,yp-b/2,zp+(H_m+H/2),'p','LineWidth',0.5);
%  alpha(drone(9:12),0.3);

% design 4 cylinder rotors and propellers individually
%rotor1(1) = surface(xcylinder+b/2,ycylinder,H_m*zcylinder+H/2,'facecolor','r');
rotor1(1) = surface(xcylinder,ycylinder,H_m*zcylinder+H/2,'facecolor','r');
rotor1(2)  = patch(xp,yp,zp+(H_m+H/2),'r','LineWidth',0.5);
%rotor1(2)  = patch(0,0,zp+(H_m+H/2),'r','LineWidth',0.5);
alpha(rotor1(1),0.7);
alpha(rotor1(2),0.3);

rotor2(1) = surface(xcylinder,ycylinder,H_m*zcylinder+H/2,'facecolor','g');
rotor2(2) = patch(xp,yp,zp+(H_m+H/2),'g','LineWidth',0.5);
alpha(rotor2(1),0.7);
alpha(rotor2(2),0.3);

rotor3(1) = surface(xcylinder,ycylinder,H_m*zcylinder+H/2,'facecolor','b');
rotor3(2) = patch(xp,yp,zp+(H_m+H/2),'b','LineWidth',0.5);
alpha(rotor3(1),0.7);
alpha(rotor3(2),0.3);

rotor4(1) = surface(xcylinder,ycylinder,H_m*zcylinder+H/2,'facecolor','k');
rotor4(2) = patch(xp,yp,zp+(H_m+H/2),'k','LineWidth',0.5);
alpha(rotor4(1),0.7);
alpha(rotor4(2),0.3);



 
 
 
 

%% create a group object and parent surface
  combinedobject = hgtransform('parent',hg );
  set(drone,'parent',combinedobject)

  combinedobject1 = hgtransform('parent',hg);
  set(rotor1,'parent',combinedobject1)

  combinedobject2 = hgtransform('parent',hg);
  set(rotor2,'parent',combinedobject2)

  combinedobject3 = hgtransform('parent',hg);
  set(rotor3,'parent',combinedobject3)

  combinedobject4 = hgtransform('parent',hg);
  set(rotor4,'parent',combinedobject4)
%  drawnow
 
%  for i = 1:250:length(x)
%      ba = plot3(x(1:i),y(1:i),z(1:i), 'b:','LineWidth',1.5);
%      
%      translation = makehgtform('translate',...
%                                [x(i) y(i) z(i)]);
%      %set(combinedobject, 'matrix',translation);
%      rotation1 = makehgtform('xrotate',(pi/180)*(roll(i)));
%      rotation2 = makehgtform('yrotate',(pi/180)*(pitch(i)));
%      rotation3 = makehgtform('zrotate',yaw(i)+(pi/4));
%      %scaling = makehgtform('scale',1-i/20);
%      set(combinedobject,'matrix',...
%           translation*rotation3*rotation2*rotation1);
%      
%      %rotor1
%      translation = makehgtform('translate',...
%                                [x(i) y(i) z(i)]);
%      rotation1 = makehgtform('xrotate',(pi/180)*(beta(1,i)));
%      rotation2 = makehgtform('yrotate',(pi/180)*(eta(1,i)));
%      rotation3 = makehgtform('zrotate',0);
%      set(combinedobject1,'matrix',...
%           translation*rotation3*rotation2*rotation1);
% 
%      %rotor2
%      translation = makehgtform('translate',...
%                                [x(i) y(i) z(i)]);
%      rotation1 = makehgtform('xrotate',(pi/180)*(beta(2,i)));
%      rotation2 = makehgtform('yrotate',(pi/180)*(eta(2,i)));
%      rotation3 = makehgtform('zrotate',0);
%      set(combinedobject2,'matrix',...
%           translation*rotation3*rotation2*rotation1);
% 
%      %rotor3
%      translation = makehgtform('translate',...
%                                [x(i) y(i) z(i)]);
%      rotation1 = makehgtform('xrotate',(pi/180)*(beta(3,i)));
%      rotation2 = makehgtform('yrotate',(pi/180)*(eta(3,i)));
%      rotation3 = makehgtform('zrotate',0);
%      set(combinedobject3,'matrix',...
%           translation*rotation3*rotation2*rotation1);
% 
%      %rotor4
%      translation = makehgtform('translate',...
%                                [x(i) y(i) z(i)]);
%      rotation1 = makehgtform('xrotate',(pi/180)*(beta(4,i)));
%      rotation2 = makehgtform('yrotate',(pi/180)*(eta(4,i)));
%      rotation3 = makehgtform('zrotate',0);
%      set(combinedobject4,'matrix',...
%           translation*rotation3*rotation2*rotation1);
% 
% 
%       %movieVector(i) =  getframe(fig1);
%         %delete(b);
%      drawnow
%     %pause(0.1);
%  end
%% No Translation
 for i = 1:250:length(x)
     ba = plot3(x(1:i),y(1:i),z(1:i), 'b:','LineWidth',1.5);
     
     
     %set(combinedobject, 'matrix',translation);
          translation = makehgtform('translate',...
                               [x(i) y(i) z(i)]);
     rotation1 = makehgtform('xrotate',(pi/180)*(roll(i)));
     rotation2 = makehgtform('yrotate',(pi/180)*(pitch(i)));
     rotation3 = makehgtform('zrotate',yaw(i)+(pi/4));
     %scaling = makehgtform('scale',1-i/20);
     set(combinedobject,'matrix',...
          translation*rotation3*rotation2*rotation1);
     
     %rotor1
     translation = makehgtform('translate',...
                                [x(i)+sqrt(b^2/8) y(i)+sqrt(b^2/8) z(i)]);
     rotation1 = makehgtform('xrotate',10*(beta(1,i)));
     rotation2 = makehgtform('yrotate',10*(eta(1,i)));
     rotation3 = makehgtform('zrotate',(pi/2));
     set(combinedobject1,'matrix',...
          translation*rotation3*rotation2*rotation1);

     %rotor2
     translation = makehgtform('translate',...
                                [x(i)-sqrt(b^2/8) y(i)-sqrt(b^2/8) z(i)]);
     rotation1 = makehgtform('xrotate',-10*(beta(2,i)));
     rotation2 = makehgtform('yrotate',-10*(eta(2,i)));
     rotation3 = makehgtform('zrotate',(pi/4));
     set(combinedobject2,'matrix',...
          translation*rotation3*rotation2*rotation1);

     %rotor3
     translation = makehgtform('translate',...
                                [x(i)-sqrt(b^2/8) y(i)+sqrt(b^2/8) z(i)]);
     rotation1 = makehgtform('xrotate',10*(beta(3,i)));
     rotation2 = makehgtform('yrotate',-10*(eta(3,i)));
     rotation3 = makehgtform('zrotate',(pi/4));
     set(combinedobject3,'matrix',...
          translation*rotation3*rotation2*rotation1);

     %rotor4
     translation = makehgtform('translate',...
                                [x(i)+sqrt(b^2/8) y(i)-sqrt(b^2/8) z(i)]);
     rotation1 = makehgtform('xrotate',-10*(beta(4,i)));
     rotation2 = makehgtform('yrotate',10*(eta(4,i)));
     rotation3 = makehgtform('zrotate',(pi/4));
     set(combinedobject4,'matrix',...
          translation*rotation3*rotation2*rotation1);
     
     quiver3(x1,y1,z1,rotor_f1(1,i),rotor_f1(2,i),rotor_f1(3,i),'r');
     hold on
     quiver3(x2,y2,z2,rotor_f2(1,i),rotor_f2(2,i),rotor_f2(3,i),'g');
     hold on
     quiver3(x3,y3,z3,rotor_f3(1,i),rotor_f3(2,i),rotor_f3(3,i),'b');
     hold on
     quiver3(x4,y4,z4,rotor_f4(1,i),rotor_f4(2,i),rotor_f4(3,i),'k');
     hold on

      %movieVector(i) =  getframe(fig1);
        %delete(b);
     drawnow
    pause(0.001);
 end
 %exportgraphics(fig1,'config1sim.pdf')

 
