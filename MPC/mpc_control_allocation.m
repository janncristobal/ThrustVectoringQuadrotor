clc
clear
close all



Ts = 0.1; % sampling time

% Parameters MPC
options = optimoptions('fmincon','Algorithm','sqp','Display','none', ...
    'MaxIterations',20);

Duration = 15; % Run for 'Duration' time units

N = 5;  % Prediction horizon (number of iterations)
m = N;  % Control horizon (number of iterations)
nu = 6; % number of virtual inputs
nx = 12; % number of states
 
Q = 100*diag([1 1 1 1 1 1 1 1 1 1 1 1]); % state weight 
R = 1*diag([0.1 0.1 0.1 0.1 0.1 0.1]); % input weight


% reference trajectory 
xd = @(t) 2*cos((pi/30)*t);
yd = @(t) 2*sin((pi/30)*t);
zd = @(t) -2*ones(1,length(t));
ud = @(t) -2*(pi/30)*sin((pi/30)*t);
vd = @(t) 2*(pi/30)*cos((pi/30)*t);
wd = @(t) zeros(1,length(t));
phid = @(t) zeros(1,length(t));
thetad = @(t) zeros(1,length(t));
psid = @(t) zeros(1,length(t));
pd = @(t) zeros(1,length(t));
qd = @(t) zeros(1,length(t));
rd = @(t) zeros(1,length(t));


x0 = [0 0 0 0 0 0 0 0 0 0 0 0]'; % Initial condition


%% Run MPC

% Prepare variables // Initialization
mass=0.468; 
g = 9.81;
Nt       = (Duration/Ts)+1;
xhat     = x0;
v0     = zeros(nu,m); 

for i=1:m
%     v0(3,i) = -mass*g;
      v0(1,i) = 0;
      v0(2,i) = 0;
      v0(3,i) = -mass*g;
      v0(4,i) = 0;
      v0(5,i) = 0;
      v0(6,i) = 0;
end

vopt_old = v0;
F_old = [0;0;0;0;0;0;0;0;0;0;0;0]*ones(1,m);
u_old = zeros(12,m);
xHistory = zeros(nx,Nt); 
xHistory(:,1) = xhat;
vHistory = zeros(nu,Nt); 
vHistory(:,1)   = v0(:,1);
uHistory = zeros(12,Nt); 
uHistory(:,1)   = u_old(:,1);
%uHistory(:,1) = [5239.4181;5239.4181;9126.6995;7860.1733;1.26987;0.8850;0.31499;0.15370;0.3475;1.1151;0.3004;0.1519];
tHistory = zeros(1,Nt); 
tHistory(1)   = 0;

F_M = zeros(nu,Nt);
% F_M(:,1) = [-91.1890;-91.1890;-450.8792;80.93030;10.2587;-91.189];
F_M(:,1) = [0;0;0;0;0;0];
FR1x = zeros(1,Nt);
FR1x(1) = 0;

Omega = zeros(m,Nt);
Omega(:,1) = 0*ones(m,1);

% Start simulation
fprintf('Simulation started.  It might take a while...\n')
% Display waitbar to show simulation progress
hbar = waitbar(0,"Simulation Progress");
tic

for k = 1:Duration/Ts 

    % Set references over optimization horizon
    tref = (k-1:k+N-2).*Ts;
    xref = [pd(tref); qd(tref); rd(tref); phid(tref); thetad(tref); psid(tref); ud(tref); vd(tref); wd(tref); xd(tref); yd(tref); zd(tref)];
    
    % NMPC with full-state feedback 
    vopt = fmincon(@(vopt) ObjectiveFCN(vopt,xhat,Ts,N,xref,v0,Q,R,Omega(:,k),tHistory(k)),v0,[],[],[],[],[],[],@(vopt) ConstraintFCN(vopt,F_old,xhat,Ts,N),options);
    
    %%%%% control allocation to compute u

    

    L = [0.225, 0.225, 0];
    kT = 2.98e-6;
    kQ = 1.140e-7;

    B = zeros(6,12);
    %Fx
B(1,1) = 1; 
B(1,4) = 1;
B(1,7) = 1; 
B(1,10) =1; 
% Fy 
B(2,2) = 1; 
B(2,5) = 1;
B(2,8) = 1; 
B(2,11) =1; 
%Fz 
B(3,3) = -1; 
B(3,6) = -1;
B(3,9) = -1; 
B(3,12) = -1; 
%Mx
B(4,3) = L(2);
B(4,6) = -L(2);
B(4,9) = -L(2);
B(4,12) = L(2);
B(4,2) = L(3);
B(4,5) = L(3);
B(4,8) = L(3);
B(4,11) = L(3);
%My
B(5,3) = -L(1);
B(5,6) = L(1);
B(5,9) = -L(1);
B(5,12) = L(1);
B(5,1) = -L(3);
B(5,4) = -L(3);
B(5,7) = -L(3);
B(5,10) = -L(3);
%Mz
B(6,2) = L(1);
B(6,5) = -L(1);
B(6,8) = L(1);
B(6,11) = -L(1);
B(6,1) = -L(2);
B(6,4) = L(2);
B(6,7) = L(2);
B(6,10) = -L(2);
B(6,3) = kQ;
B(6,6) = kQ;
B(6,9) = -kQ;
B(6,12) = -kQ;



%     %solve for u using pseudo inverse
%     F = pinv(B)*vopt; 
%     u_R = [F(1,:);F(2,:);F(3,:);-F(4,:);-F(5,:);F(6,:);F(7,:);-F(8,:);-F(9,:);-F(10,:);F(11,:);-F(12,:)];

F_body = pinv(B)*vopt;  

C_R1_B = diag([ 1, 1, 1]);
C_R2_B = diag([-1,-1, 1]);
C_R3_B = diag([ 1,-1,-1]);
C_R4_B = diag([-1, 1,-1]);

F_rotor1 = C_R1_B * F_body(1:3,1);
F_rotor2 = C_R2_B * F_body(4:6,1);
F_rotor3 = C_R3_B * F_body(7:9,1);
F_rotor4 = C_R4_B * F_body(10:12,1);

F_rotor = [F_rotor1;F_rotor2;F_rotor3;F_rotor4];

e1 =  asin((F_rotor1(1))/norm(F_rotor1));
e2 =  asin((F_rotor2(1))/norm(F_rotor2));
e3 = -asin((F_rotor3(1))/norm(F_rotor3));
e4 = -asin((F_rotor4(1))/norm(F_rotor4));

b1 = -asin((F_rotor1(2))/(cos(e1)*norm(F_rotor1)));
b2 = -asin((F_rotor2(2))/(cos(e2)*norm(F_rotor2)));
b3 =  asin((F_rotor3(2))/(cos(e3)*norm(F_rotor3)));
b4 =  asin((F_rotor4(2))/(cos(e4)*norm(F_rotor4)));

w1 = (30/pi)*sqrt(norm(F_rotor1)/(kT));
w2 = (30/pi)*sqrt(norm(F_rotor2)/(kT));
w3 = (30/pi)*sqrt(norm(F_rotor3)/(kT));
w4 = (30/pi)*sqrt(norm(F_rotor4)/(kT));

u = [w1,w2,w3,w4,b1,b2,b3,b4,e1,e2,e3,e4]';


% e1 = asin((u_R(1,1))/(sqrt(u_R(1,1)^2+u_R(2,1)^2+u_R(3,1)^2)));
% e2 = asin((u_R(4,1))/(sqrt(u_R(4,1)^2+u_R(5,1)^2+u_R(6,1)^2)));
% e3 = -asin((u_R(7,1))/(sqrt(u_R(7,1)^2+u_R(8,1)^2+u_R(9,1)^2)));
% e4 = -asin((u_R(10,1))/(sqrt(u_R(10,1)^2+u_R(11,1)^2+u_R(12,1)^2)));
% 
% b1 = -asin((u_R(2,1))/(cos(e1(1,1))*sqrt(u_R(1,1)^2+u_R(2,1)^2+u_R(3,1)^2)));
% b2 = -asin((u_R(5,1))/(cos(e2(1,1))*sqrt(u_R(4,1)^2+u_R(5,1)^2+u_R(6,1)^2)));
% b3 = asin((u_R(8,1))/(cos(e3(1,1))*sqrt(u_R(7,1)^2+u_R(8,1)^2+u_R(9,1)^2)));
% b4 = asin((u_R(11,1))/(cos(e4(1,1))*sqrt(u_R(10,1)^2+u_R(11,1)^2+u_R(12,1)^2)));
% 
% % unit is rpm
% w1 = (30/pi)*sqrt(sqrt(u_R(1,1)^2+u_R(2,1)^2+u_R(3,1)^2)/(kT));
% w2 = (30/pi)*sqrt(sqrt(u_R(4,1)^2+u_R(5,1)^2+u_R(6,1)^2)/(kT));
% w3 = (30/pi)*sqrt(sqrt(u_R(7,1)^2+u_R(8,1)^2+u_R(9,1)^2)/(kT));
% w4 = (30/pi)*sqrt(sqrt(u_R(10,1)^2+u_R(11,1)^2+u_R(12,1)^2)/(kT));
% 
% 
%     u = [w1,w2,w3,w4,b1,b2,b3,b4,e1,e2,e3,e4]';






% for i = 1:N
% 
% e1(1,i) = asin((u_R(1,i))/(sqrt(u_R(1,i)^2+u_R(2,i)^2+u_R(3,i)^2)));
% e2(1,i) = asin((u_R(4,i))/(sqrt(u_R(4,i)^2+u_R(5,i)^2+u_R(6,i)^2)));
% e3(1,i) = -asin((u_R(7,i))/(sqrt(u_R(7,i)^2+u_R(8,i)^2+u_R(9,i)^2)));
% e4(1,i) = -asin((u_R(10,i))/(sqrt(u_R(10,i)^2+u_R(11,i)^2+u_R(12,i)^2)));
% 
% b1(1,i) = -asin((u_R(2,i))/(cos(e1(1,i))*sqrt(u_R(1,i)^2+u_R(2,i)^2+u_R(3,i)^2)));
% b2(1,i) = -asin((u_R(5,i))/(cos(e2(1,i))*sqrt(u_R(4,i)^2+u_R(5,i)^2+u_R(6,i)^2)));
% b3(1,i) = asin((u_R(8,i))/(cos(e3(1,i))*sqrt(u_R(7,i)^2+u_R(8,i)^2+u_R(9,i)^2)));
% b4(1,i) = asin((u_R(11,i))/(cos(e4(1,i))*sqrt(u_R(10,i)^2+u_R(11,i)^2+u_R(12,i)^2)));
% 
% % unit is rpm
% w1(1,i) = (30/pi)*sqrt(sqrt(u_R(1,i)^2+u_R(2,i)^2+u_R(3,i)^2)/(kT));
% w2(1,i) = (30/pi)*sqrt(sqrt(u_R(4,i)^2+u_R(5,i)^2+u_R(6,i)^2)/(kT));
% w3(1,i) = (30/pi)*sqrt(sqrt(u_R(7,i)^2+u_R(8,i)^2+u_R(9,i)^2)/(kT));
% w4(1,i) = (30/pi)*sqrt(sqrt(u_R(10,i)^2+u_R(11,i)^2+u_R(12,i)^2)/(kT));
% 
% 
%     u(:,i) = [w1(1,i),w2(1,i),w3(1,i),w4(1,i),b1(1,i),b2(1,i),b3(1,i),b4(1,i),e1(1,i),e2(1,i),e3(1,i),e4(1,i)]';
%    
%     
% 
% end



    % Integrate system
    % update initial condition
    opts = odeset('RelTol',1e-12,'AbsTol',1e-12);
    [TOUT,xhat] = ode45(@(t,xhat) TVequationsOfMotion(t,xhat,u(:,1)),[(k-1)*Ts (k-1)*Ts+Ts], xhat); % apply the first optimal input to the system
    v0 = [vopt(:,2:size(vopt,2)) vopt(:,size(vopt,2))];
    vopt_old = vopt;
    F_old = F_body;
  
    Omega(:,k+1) = (w1-w2+w3-w4)';

    global F_x F_y F_z M_x M_y M_z min1 max1

    F_M(:,k+1) = [F_x;F_y;F_z;M_x;M_y;M_z];
    con1(:,k+1) = min1;
    con2(:,k+1) = max1;


    % store date
    xhat = xhat(end,:)';
    xHistory(:,k+1) = xhat;
    vHistory(:,k+1) = vopt(:,1);
    uHistory(:,k+1) = u(:,1);
    fHistory(:,k+1) = F_body(:,1);
    tHistory(:,k+1) = k*Ts;
    
    % Update waitbar
    waitbar(k*Ts/Duration,hbar);
    
end
close(hbar)
tElapsed = toc;
fprintf('Simulation finished!\n')
% Close waitbar 

% figure
% 
% plot(fHistory(1,:))
% hold on
% plot(con1(1,:),'r')
% hold on
% plot(con2(1,:),'g')





%% plot

% Plot the states.
time = 0:Ts:Duration;
yreftot = [pd(time); qd(time); rd(time); phid(time); thetad(time); psid(time); ud(time); vd(time); wd(time); xd(time); yd(time); zd(time)];

figure
subplot(4,3,1)
plot(tHistory,xHistory(1,:),'b')
hold on
plot(time,yreftot(1,:),'r')
grid on
% xlabel('time')
% ylabel('P')
%legend('actual','reference','Location','southeast')
title('p')

subplot(4,3,2)
plot(tHistory,xHistory(2,:),'b')
hold on
plot(time,yreftot(2,:),'r')
grid on
% xlabel('time')
% ylabel('y')
%legend('actual','reference','Location','southeast')
title('Q')

subplot(4,3,3)
plot(tHistory,xHistory(3,:),'b')
hold on
plot(time,yreftot(3,:),'r')
grid on
% xlabel('time')
% ylabel('z')
%legend('actual','reference','Location','southeast')
title('R')

subplot(4,3,4)
plot(tHistory,xHistory(4,:),'b')
hold on
plot(time,yreftot(4,:),'r')
grid on
% xlabel('time')
% ylabel('phi')
%legend('actual','reference','Location','southeast')
title('phi')

subplot(4,3,5)
plot(tHistory,xHistory(5,:),'b')
hold on
plot(time,yreftot(5,:),'r')
grid on
% xlabel('time')
% ylabel('theta')
%legend('actual','reference','Location','southeast')
title('theta')

subplot(4,3,6)
hold on
plot(tHistory,xHistory(6,:),'b')
hold on
plot(time,yreftot(6,:),'r')
grid on
% xlabel('time')
% ylabel('psi')
%legend('actual','reference','Location','southeast')
title('psi')

subplot(4,3,7)
plot(tHistory,xHistory(7,:),'b')
hold on
plot(time,yreftot(7,:),'r')
grid on
% xlabel('time')
% ylabel('psi')
%legend('actual','reference','Location','southeast')
title('u')

subplot(4,3,8)
plot(tHistory,xHistory(8,:),'b')
hold on
plot(time,yreftot(8,:),'r')
grid on
% xlabel('time')
% ylabel('psi')
%legend('actual','reference','Location','southeast')
title('v')

subplot(4,3,9)
plot(tHistory,xHistory(9,:),'b')
hold on
plot(time,yreftot(9,:),'r')
grid on
% xlabel('time')
% ylabel('psi')
%legend('actual','reference','Location','southeast')
title('w')

subplot(4,3,10)
plot(tHistory,xHistory(10,:),'b')
hold on
plot(time,yreftot(10,:),'r')
grid on
% xlabel('time')
% ylabel('psi')
%legend('actual','reference','Location','southeast')
title('x')

subplot(4,3,11)
plot(tHistory,xHistory(11,:),'b')
hold on
plot(time,yreftot(11,:),'r')
grid on
% xlabel('time')
% ylabel('psi')
%legend('actual','reference','Location','southeast')
title('y')

subplot(4,3,12)
plot(tHistory,xHistory(12,:),'b')
hold on
plot(time,yreftot(12,:),'r')
grid on
% xlabel('time')
% ylabel('psi')
%legend('actual','reference','Location','southeast')
title('z')


%%% plot trajectory
figure
plot3(xHistory(10,:),xHistory(11,:),xHistory(12,:),'b')
hold on
plot3(yreftot(10,:),yreftot(11,:),yreftot(12,:),'r')


% % Plot the virtual control of mpc and plant
% figure
% 
% subplot(4,3,1)
% plot(F_M(1,:),'r')
% grid on
% title('Fxplant')
% subplot(4,3,4)
% plot(vHistory(1,:))
% grid on
% title('Fxmpc')
% 
% subplot(4,3,2)
% plot(F_M(2,:),'r')
% grid on
% title('Fyplant')
% subplot(4,3,5)
% plot(vHistory(2,:))
% grid on
% title('Fympc')
% 
% 
% subplot(4,3,3)
% plot(F_M(3,:),'r')
% grid on
% title('Fzplant')
% subplot(4,3,6)
% plot(vHistory(3,:))
% grid on
% title('Fzmpc')
% 
% 
% subplot(4,3,7)
% plot(F_M(4,:),'r')
% grid on
% title('Mphiplant')
% subplot(4,3,10)
% plot(vHistory(4,:))
% grid on
% title('Mphimpc')
% 
% 
% subplot(4,3,8)
% plot(F_M(5,:),'r')
% grid on
% title('Mthetaplant')
% subplot(4,3,11)
% plot(vHistory(5,:))
% grid on
% title('Mthetampc')
% 
% 
% subplot(4,3,9)
% plot(F_M(6,:),'r')
% grid on
% title('Mpsiplant')
% subplot(4,3,12)
% plot(vHistory(6,:))
% grid on
% title('Mpsimpc')

figure

subplot(2,3,1)
plot(F_M(1,:),'r')
grid on
hold on
plot(vHistory(1,:),'b')
legend('Fxplant','Fxmpc')

subplot(2,3,2)
plot(F_M(2,:),'r')
grid on
hold on
plot(vHistory(2,:),'b')
legend('Fyplant','Fympc')

subplot(2,3,3)
plot(F_M(3,:),'r')
grid on
hold on
plot(vHistory(3,:),'b')
legend('Fzplant','Fzmpc')

subplot(2,3,4)
plot(F_M(4,:),'r')
grid on
hold on
plot(vHistory(4,:),'b')
legend('Mphiplant','Mphimpc')

subplot(2,3,5)
plot(F_M(5,:),'r')
grid on
hold on
plot(vHistory(5,:),'b')
legend('Mthetaplant','Mthetampc')

subplot(2,3,6)
plot(F_M(6,:),'r')
grid on
hold on
plot(vHistory(6,:),'b')
legend('Mpsiplant','Mpsimpc')

% % plot virtual control signal 
figure

subplot(3,2,1)
hold on
plot(tHistory,vHistory(1,:))
grid on
xlabel('time')
title('Fx')

subplot(3,2,2)
hold on
plot(tHistory,vHistory(2,:))
grid on
xlabel('time')
title('Fy')

subplot(3,2,3)
hold on
plot(tHistory,vHistory(3,:))
grid on
xlabel('time')
title('Fz')

subplot(3,2,4)
hold on
plot(tHistory,vHistory(4,:))
grid on
xlabel('time')
title('Mphi')

subplot(3,2,5)
hold on
plot(tHistory,vHistory(5,:))
grid on
xlabel('time')
title('Mtheta')

subplot(3,2,6)
hold on
plot(tHistory,vHistory(6,:))
grid on
xlabel('time')
title('Mpsi')

% % Plot the control variables.
figure
subplot(4,3,1)
hold on
plot(tHistory,uHistory(1,:))
grid on
xlabel('time')
title('w1')

subplot(4,3,2)
hold on
plot(tHistory,uHistory(2,:))
grid on
xlabel('time')
title('w2')


subplot(4,3,3)
hold on
plot(tHistory,uHistory(3,:))
grid on
xlabel('time')
title('w3')


subplot(4,3,4)
hold on
plot(tHistory,uHistory(4,:))
grid on
xlabel('time')
title('w4')


subplot(4,3,5)
hold on
plot(tHistory,uHistory(5,:))
grid on
xlabel('time')
title('b1')


subplot(4,3,6)
hold on
plot(tHistory,uHistory(6,:))
grid on
xlabel('time')
title('b2')


subplot(4,3,7)
hold on
plot(tHistory,uHistory(7,:))
grid on
xlabel('time')
title('b3')

subplot(4,3,8)
hold on
plot(tHistory,uHistory(8,:))
grid on
xlabel('time')
title('b4')


subplot(4,3,9)
hold on
plot(tHistory,uHistory(9,:))
grid on
xlabel('time')
title('e1')


subplot(4,3,10)
hold on
plot(tHistory,uHistory(10,:))
grid on
xlabel('time')
title('e2')


subplot(4,3,11)
hold on
plot(tHistory,uHistory(11,:))
grid on
xlabel('time')
title('e3')


subplot(4,3,12)
hold on
plot(tHistory,uHistory(12,:))
grid on
xlabel('time')
title('e4')




%% cost
function J = ObjectiveFCN(v,x,Ts,N,xref,v0,Q,R,Omegaold,t0)

% Set initial plant states, controller output and cost
xk = x;

J = 0;

% Loop through each prediction step
for k=1:N

    % Obtain plant state at next prediction step
    [TOUT,xk1] = ode45(@(t,xk) system(t,xk,v(:,k)),[t0 t0+Ts], xk);
%     [TOUT,xk1] = ode45(@(t,xk) system(t,xk,v(:,k)),[(k-1)*Ts (k-1)*Ts+Ts], xk);
    
    % Accumulate state tracking and input cost from x(k+1) to x(k+N) and u(k+1) to u(k+N)
    J = J + (xk1(end,:)'-xref(:,k))'*Q*(xk1(end,:)'-xref(:,k)) + v(:,k)'*R*v(:,k);
    
 
    % Update xk and uk for the next prediction step
    xk = xk1(end,:)';
    
end
end 

%% constraints
function [c, ceq] = ConstraintFCN(v,Fold,x,Ts,N)

L = [0.225, 0.225, 0];
kT = 2.98e-6;
kQ = 1.140e-7;

B = zeros(6,12);

B(1,1) = 1; 
B(1,4) = 1;
B(1,7) = 1; 
B(1,10) =1; 
% Fy 
B(2,2) = 1; 
B(2,5) = 1;
B(2,8) = 1; 
B(2,11) =1; 
%Fz 
B(3,3) = -1; 
B(3,6) = -1;
B(3,9) = -1; 
B(3,12) = -1; 
%Mx
B(4,3) = L(2);
B(4,6) = -L(2);
B(4,9) = -L(2);
B(4,12) = L(2);
B(4,2) = L(3);
B(4,5) = L(3);
B(4,8) = L(3);
B(4,11) = L(3);
%My
B(5,3) = -L(1);
B(5,6) = L(1);
B(5,9) = -L(1);
B(5,12) = L(1);
B(5,1) = -L(3);
B(5,4) = -L(3);
B(5,7) = -L(3);
B(5,10) = -L(3);
%Mz
B(6,2) = L(1);
B(6,5) = -L(1);
B(6,8) = L(1);
B(6,11) = -L(1);
B(6,1) = -L(2);
B(6,4) = L(2);
B(6,7) = L(2);
B(6,10) = -L(2);
B(6,3) = kQ;
B(6,6) = kQ;
B(6,9) = -kQ;
B(6,12) = -kQ;

F = pinv(B)*v;
 
%%%%%%%%%%% dynamic constraints
global min1 max1

for i=1:N

    min1(i,1) = max(-13,(Fold(1,1) - 0.5));
    max1(i,1) = min(13,(Fold(1,1) + 0.5));

    c1(2*i-1) = max(-13,(Fold(1,1) - 0.5)) - F(1,i); %min
    c1(2*i) = F(1,i) - min(13,(Fold(1,1) + 0.5)); %max

    c2(2*i-1) = max(-13,(Fold(2,1) - 0.5)) - F(2,i);
    c2(2*i) = F(2,i) - min(13,(Fold(2,1) + 0.5));

    c3(2*i-1) = max(-13,(Fold(3,1) - 0.5)) - F(3,i);
    c3(2*i) = F(3,i) - min(13,(Fold(3,1) + 0.5));

    c4(2*i-1) = max(-13,(Fold(4,1) - 0.5)) - F(4,i);
    c4(2*i) = F(4,i) - min(13,(Fold(4,1) + 0.5));

    c5(2*i-1) = max(-13,(Fold(5,1) - 0.5)) - F(5,i);
    c5(2*i) = F(5,i) - min(13,(Fold(5,1) + 0.5));

    c6(2*i-1) =  max(-13,(Fold(6,1) - 0.5)) - F(6,i);
    c6(2*i) = F(6,i) - min(13,(Fold(6,1) + 0.5));

    c7(2*i-1) = max(-13,(Fold(7,1) - 0.5)) - F(7,i);
    c7(2*i) = F(7,i) - min(13,(Fold(7,1) + 0.5));

    c8(2*i-1) = max(-13,(Fold(8,1) - 0.5)) - F(8,i);
    c8(2*i) = F(8,i) - min(13,(Fold(8,1) + 0.5));

    c9(2*i-1) = max(-13,(Fold(9,1) - 0.5)) - F(9,i);
    c9(2*i) = F(9,i) - min(13,(Fold(9,1) + 0.5));

    c10(2*i-1) = max(-13,(Fold(10,1) - 0.5)) - F(10,i);
    c10(2*i) = F(10,i) - min(13,(Fold(10,1) + 0.5));

    c11(2*i-1) = max(-13,(Fold(11,1) - 0.5)) - F(11,i);
    c11(2*i) = F(11,i) - min(13,(Fold(11,1) + 0.5));

    c12(2*i-1) = max(-13,(Fold(12,1) - 0.5)) - F(12,i);
    c12(2*i) = F(12,i) - min(13,(Fold(12,1) + 0.5));

    if i<N

        Fold = F(:,i);
        
    end


end 
 


    c = [c1;c2;c3;c4;c5;c6;c7;c8;c9;c10;c11;c12];
    ceq  = [];


% %%%%%%%%%%%% static constraint
% c =[];
% 
% for i=1:N
%     c1 = F(1,i)-18;
%     c2 = -18-F(1,i);
%     c3 = F(2,i)-18;
%     c4 = -18-F(2,i);
%     c5 = F(3,i)-18;
%     c6 = -18-F(3,i);
%     c7 = F(4,i)-18;
%     c8 = -18-F(4,i);
%     c9 = F(5,i)-18;
%     c10 = -18-F(5,i);
%     c11 = F(6,i)-18;
%     c12 = -18-F(6,i);
%     c13 = F(7,i)-18;
%     c14 = -18-F(7,i);
%     c15 = F(8,i)-18;
%     c16 = -18-F(8,i);
%     c17 = F(9,i)-18;
%     c18 = -18-F(9,i);
%     c19 = F(10,i)-18;
%     c20 = -18-F(10,i);
%     c21 = F(11,i)-18;
%     c22 = -18-F(11,i);
%     c23 = F(12,i)-18;
%     c24 = -18-F(12,i);
% 
%     cnew = [c1;c2;c3;c4;c5;c6;c7;c8;c9;c10;c11;c12;c13;c14;c15;c16;c17;c18;c19;c20;c21;c22;c23;c24];
%     c = [c cnew];
%     ceq  = [];
% 
% end 

     
end 
%% UAV mpdel for mpc
function dx = system(t,x,v,Omega)

X = x(10);Y = x(11);Z = x(12);
%Velocity
U = x(7);V = x(8);W = x(9);
%Euler Angles
Phi = x(4);The = x(5);Psi = x(6);
%Body Rates
P = x(1);Q = x(2);R = x(3);

%inputs 
FBx = v(1);
FBy = v(2);
FBz = v(3);
MPhi = v(4);
MThe = v(5);
MPsi = v(6);



% System coefficients

m=0.468; 
g = 9.81;

Ixx=4.856e-03;
Iyy=4.856e-03;
Izz=8.801e-03;


Ax=0.3;
Ay=0.3;
Az=0.25;
Ar=0.2;
Ir=3.357e-05;

%
dP = (1/Ixx)*((Iyy-Izz)*Q*R+MPhi-Ar*P);
dQ = (1/Iyy)*((Ixx-Izz)*P*R+MThe-Ar*Q);
dR = (1/Izz)*((Iyy-Ixx)*Q*P+MPsi-Ar*R);

dPhi= P + sin(Phi)*tan(The)*Q + cos(Phi)*tan(The)*R;
dThe= cos(Phi)*Q - sin(Phi)*R;
dPsi= sin(Phi)/cos(The)*Q + cos(Phi)/cos(The)*R;

dU = (FBx*cos(Psi)*cos(The)...
    + FBy*(cos(Phi)*sin(Psi) + cos(Psi)*sin(Phi)*sin(The))...
    + FBz*(sin(Phi)*sin(Psi) - cos(Phi)*cos(Psi)*sin(The))...
    - Ax*U)/m;
dV = (-FBx*cos(The)*sin(Psi)...
    + FBy*(cos(Phi)*cos(Psi) - sin(Phi)*sin(Psi)*sin(The))...
    + FBz*(cos(Psi)*sin(Phi) + cos(Phi)*sin(Psi)*sin(The))...
    - Ay*V)/m;
dW = (g + (FBx*sin(The)...
    - FBy*cos(The)*sin(Phi)...
    + FBz*cos(Phi)*cos(The)...
    - Az*W)/m);

dX = U;
dY = V;
dZ = W;
dx = [dP dQ dR dPhi dThe dPsi dU dV dW dX dY dZ]';

end

%% main plant 
% TV EquationOfMotion

function x_dot = TVequationsOfMotion(t,x,u)

%states 
%Position       
X = x(10);Y = x(11);Z = x(12);
%Velocity
U = x(7);V = x(8);W = x(9);
%Euler Angles
Phi = x(4);The = x(5);Psi = x(6);
%Body Rates
P = x(1);Q = x(2);R = x(3);

%inputs 
%Rotor Speed 
w = [u(1),u(2),u(3),u(4)];
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
g = 9.81;

Ixx=4.856e-03;Iyy=4.856e-03;Izz=8.801e-03;

Ir=3.357e-05;
Ax=0.3; Ay=0.3; Az=0.25; Ar=0.2;

T1= k*(w(1)*pi/30)^2;
T2= k*(w(2)*pi/30)^2;
T3= k*(w(2)*pi/30)^2;
T4= k*(w(2)*pi/30)^2;

T = [-T1,-T2,T3,T4]; % Rotor Thrust

%rotor forces
for i =1:4
    F_R(:,i) = [-T(i)*sin(eta(i));
                +T(i)*cos(eta(i))*sin(beta(i));
                +T(i)*cos(beta(i))*cos(eta(i))];
end

%transform rotor forces to body frame

C_R1_B = diag([ 1, 1, 1]);
C_R2_B = diag([-1,-1, 1]);
C_R3_B = diag([ 1,-1,-1]);
C_R4_B = diag([-1, 1,-1]);

F_B(:,1) = C_R1_B' * F_R(:,1);
F_B(:,2) = C_R2_B' * F_R(:,2);
F_B(:,3) = C_R3_B' * F_R(:,3);
F_B(:,4) = C_R4_B' * F_R(:,4);

% F_B(:,1) = F_R(:,1);
% F_B(:,2) = [-1,0,0;0,-1,0;0,0,1]*F_R(:,2);
% F_B(:,3) = [1,0,0;0,-1,0;0,0,-1]*F_R(:,3);
% F_B(:,4) = [-1,0,0;0,1,0;0,0,-1]*F_R(:,4);


%body forces 
FBx = F_B(1,1)+F_B(1,2)+F_B(1,3)+F_B(1,4);
FBy = F_B(2,1)+F_B(2,2)+F_B(2,3)+F_B(2,4);
FBz = F_B(3,1)+F_B(3,2)+F_B(3,3)+F_B(3,4);




MPhi = l(2)*(F_B(3,1)+F_B(3,2)-F_B(3,3)-F_B(3,4))...
    + l(3)*(F_B(2,1)+F_B(2,2)+F_B(2,3)+F_B(2,4));

MThe = l(1)*(-F_B(3,1)+F_B(3,2)-F_B(3,3)+F_B(3,4))...
    + l(3)*(-F_B(2,1)-F_B(2,2)-F_B(2,3)-F_B(2,4));

MPsi = (kQ)*(F_B(3,1)+F_B(3,2)-F_B(3,3)-F_B(3,4))...
    + l(1)*(F_B(2,1)-F_B(2,2)+F_B(2,3)-F_B(2,4))...
    + l(2)*(-F_B(1,1)+F_B(1,2)+F_B(1,3)-F_B(1,4));

global F_x F_y F_z M_x M_y M_z FR

F_x = FBx;
F_y = FBy;
F_z = FBz;
M_x = MPhi;
M_y = MThe;
M_z = MPsi;
FR = F_R(2,3); 

Omega = w(1)-w(2)+w(3)-w(4);

% MPhi = -MPhi;
% MThe = -MThe;

dP = (1/Ixx)*((Iyy-Izz)*Q*R+MPhi-Ir*Q*Omega-Ar*P);
dQ = (1/Iyy)*((Ixx-Izz)*P*R+MThe-Ir*P*Omega-Ar*Q);
dR = (1/Izz)*((Iyy-Ixx)*Q*P+MPsi-Ar*R);

dPhi= P + sin(Phi)*tan(The)*Q + cos(Phi)*tan(The)*R;
dThe= cos(Phi)*Q - sin(Phi)*R;
dPsi= sin(Phi)/cos(The)*Q + cos(Phi)/cos(The)*R;

dU = (FBx*cos(Psi)*cos(The)...
    + FBy*(cos(Phi)*sin(Psi) + cos(Psi)*sin(Phi)*sin(The))...
    + FBz*(sin(Phi)*sin(Psi) - cos(Phi)*cos(Psi)*sin(The))...
    - Ax*U)/m;
dV = (-FBx*cos(The)*sin(Psi)...
    + FBy*(cos(Phi)*cos(Psi) - sin(Phi)*sin(Psi)*sin(The))...
    + FBz*(cos(Psi)*sin(Phi) + cos(Phi)*sin(Psi)*sin(The))...
    - Ay*V)/m;
dW = (g + (FBx*sin(The)...
    - FBy*cos(The)*sin(Phi)...
    + FBz*cos(Phi)*cos(The)...
    - Az*W)/m);

dX = U;
dY = V;
dZ = W;
x_dot = [dP dQ dR dPhi dThe dPsi dU dV dW dX dY dZ]';

end
