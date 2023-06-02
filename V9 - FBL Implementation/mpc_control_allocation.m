clc
clear
close all



Ts = 0.01; % sampling time

% Parameters MPC
options = optimoptions('fmincon','Algorithm','sqp','Display','none', ...
    'MaxIterations',400);

Duration = 5; % Run for 'Duration' time units

N = 5;  % Prediction horizon (number of iterations)
m = N;  % Control horizon (number of iterations)
nu = 6; % number of virtual inputs
nx = 12; % number of states
 
Q = 10*diag([10 10 10 10 10 10 10 10 10 10 10 30]); % state weight 
R = 0.1*diag([0.1 0.1 0.1 0.1 0.1 0.1]); % input weight


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
      v0(1,i) = -10;
      v0(2,i) = -10;
      v0(3,i) = -10;
      v0(4,i) = -10;
      v0(5,i) = -10;
      v0(6,i) = -10;
end

vopt_old = v0;
uopt_old = 100*[5.9257;5.9257;5.9257;5.9257;0;0;0;0;0;0;0;0]*ones(1,m);
xHistory = zeros(nx,Nt); 
xHistory(:,1) = xhat;
vHistory = zeros(nu,Nt); 
vHistory(:,1)   = v0(:,1);
uHistory = zeros(12,Nt); 
uHistory(:,1)   = uopt_old(:,1);
tHistory = zeros(1,Nt); 
tHistory(1)   = 0;


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
    vopt = fmincon(@(vopt) ObjectiveFCN(vopt,xhat,Ts,N,xref,v0,Q,R),v0,[],[],[],[],[],[],@(vopt) ConstraintFCN(vopt,uopt_old,xhat,Ts,N),options);
    
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


    %solve for u using pseudo inverse
    F = pinv(B)*vopt; 
    %F = [F1x,F1y,F1z,TF2x,F2y,F2z,F3x,F3y,F3z,F4x,F4y,F4z]
    %u_R = [-F(1,:);F(2,:);F(3,:);-F(4,:);F(5,:);F(6,:);-F(7,:);F(8,:);F(9,:);-F(10,:);F(11,:);F(12,:)];
    u_R = [F(1,:);F(2,:);F(3,:);-F(4,:);-F(5,:);F(6,:);F(7,:);-F(8,:);-F(9,:);-F(10,:);F(11,:);-F(12,:)];

for i = 1:N

e1(1,i) = asin((u_R(1,i))/(sqrt(u_R(1,i)^2+u_R(2,i)^2+u_R(3,i)^2)));
e2(1,i) = asin((u_R(4,i))/(sqrt(u_R(4,i)^2+u_R(5,i)^2+u_R(6,i)^2)));
e3(1,i) = -asin((u_R(7,i))/(sqrt(u_R(7,i)^2+u_R(8,i)^2+u_R(9,i)^2)));
e4(1,i) = -asin((u_R(10,i))/(sqrt(u_R(10,i)^2+u_R(11,i)^2+u_R(12,i)^2)));

b1(1,i) = -asin((u_R(2,i))/(cos(e1(1,i))*sqrt(u_R(1,i)^2+u_R(2,i)^2+u_R(3,i)^2)));
b2(1,i) = -asin((u_R(5,i))/(cos(e2(1,i))*sqrt(u_R(4,i)^2+u_R(5,i)^2+u_R(6,i)^2)));
b3(1,i) = asin((u_R(8,i))/(cos(e3(1,i))*sqrt(u_R(7,i)^2+u_R(8,i)^2+u_R(9,i)^2)));
b4(1,i) = asin((u_R(11,i))/(cos(e4(1,i))*sqrt(u_R(10,i)^2+u_R(11,i)^2+u_R(12,i)^2)));

w1(1,i) = (30/pi)*sqrt(sqrt(u_R(1,i)^2+u_R(2,i)^2+u_R(3,i)^2)/(kT));
w2(1,i) = (30/pi)*sqrt(sqrt(u_R(4,i)^2+u_R(5,i)^2+u_R(6,i)^2)/(kT));
w3(1,i) = (30/pi)*sqrt(sqrt(u_R(7,i)^2+u_R(8,i)^2+u_R(9,i)^2)/(kT));
w4(1,i) = (30/pi)*sqrt(sqrt(u_R(10,i)^2+u_R(11,i)^2+u_R(12,i)^2)/(kT));




    uopt(:,i) = [w1(1,i),w2(1,i),w3(1,i),w4(1,i),b1(1,i),b2(1,i),b3(1,i),b4(1,i),e1(1,i),e2(1,i),e3(1,i),e4(1,i)]';


end



% Go to Simulink 

    % Integrate system
    % update initial condition
    opts = odeset('RelTol',1e-12,'AbsTol',1e-12);
    [TOUT,xhat] = ode45(@(t,xhat) TVequationsOfMotion(t,xhat,uopt(:,1)),[(k-1)*Ts (k-1)*Ts+Ts], xhat); % apply the first optimal input to the system
    v0 = [vopt(:,2:size(vopt,2)) vopt(:,size(vopt,2))];
    vopt_old = vopt;
    uopt_old = uopt;

    % store date
    xhat = xhat(end,:)';
    xHistory(:,k+1) = xhat;
    vHistory(:,k+1) = vopt(:,1);
    uHistory(:,k+1) = uopt(:,1);
    tHistory(:,k+1) = k*Ts;
    
    % Update waitbar
    waitbar(k*Ts/Duration,hbar);
    
end
close(hbar)
tElapsed = toc;
fprintf('Simulation finished!\n')
% Close waitbar 



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


% % Plot the virtual control variables.
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
function J = ObjectiveFCN(v,x,Ts,N,xref,v0,Q,R)

% Set initial plant states, controller output and cost
xk = x;

J = 0;

% Loop through each prediction step
for k=1:N

    % Obtain plant state at next prediction step
    [TOUT,xk1] = ode45(@(t,xk) system(t,xk,v(:,k)),[(k-1)*Ts (k-1)*Ts+Ts], xk);
    
    % Accumulate state tracking and input cost from x(k+1) to x(k+N) and u(k+1) to u(k+N)
    J = J + (xk1(end,:)'-xref(:,k))'*Q*(xk1(end,:)'-xref(:,k)) + v(:,k)'*R*v(:,k);
    
 
    % Update xk and uk for the next prediction step
    xk = xk1(end,:)';
    
end
end 

%% constraints
function [c, ceq] = ConstraintFCN(v,uold,x,Ts,N)

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



% %Fx
% B(1,1) = 1; 
% B(1,4) = -1;
% B(1,7) = 1; 
% B(1,10) =-1; 
% % Fy 
% B(2,2) = 1; 
% B(2,5) = -1;
% B(2,8) = -1; 
% B(2,11) =1; 
% %Fz 
% B(3,3) = 1; 
% B(3,6) = 1;
% B(3,9) = -1; 
% B(3,12) = -1; 
% %Mx
% B(4,3) = -L(2);
% B(4,6) = L(2);
% B(4,9) = L(2);
% B(4,12) = -L(2);
% B(4,2) = L(3);
% B(4,5) = -L(3);
% B(4,8) = L(3);
% B(4,11) = -L(3);
% %My
% B(5,3) = L(1);
% B(5,6) = -L(1);
% B(5,9) = L(1);
% B(5,12) = -L(1);
% B(5,1) = -L(3);
% B(5,4) = -L(3);
% B(5,7) = -L(3);
% B(5,10) = -L(3);
% %Mz
% B(6,2) = 0;
% B(6,5) = 0;
% B(6,8) = 0;
% B(6,11) = 0;
% B(6,1) = 0;
% B(6,4) = 0;
% B(6,7) = 0;
% B(6,10) = 0;
% B(6,3) = -kQ;
% B(6,6) = kQ;
% B(6,9) = -kQ;
% B(6,12) = kQ;

F = pinv(B)*v;
% u_R = [-F(1,:);F(2,:);F(3,:);-F(4,:);F(5,:);F(6,:);-F(7,:);F(8,:);F(9,:);-F(10,:);F(11,:);F(12,:)];
u_R = [F(1,:);F(2,:);F(3,:);-F(4,:);-F(5,:);F(6,:);F(7,:);-F(8,:);-F(9,:);-F(10,:);F(11,:);-F(12,:)];


for i=1:N

e1(1,i) = asin((u_R(1,i))/(sqrt(u_R(1,i)^2+u_R(2,i)^2+u_R(3,i)^2)));
e2(1,i) = asin((u_R(4,i))/(sqrt(u_R(4,i)^2+u_R(5,i)^2+u_R(6,i)^2)));
e3(1,i) = -asin((u_R(7,i))/(sqrt(u_R(7,i)^2+u_R(8,i)^2+u_R(9,i)^2)));
e4(1,i) = -asin((u_R(10,i))/(sqrt(u_R(10,i)^2+u_R(11,i)^2+u_R(12,i)^2)));

b1(1,i) = -asin((u_R(2,i))/(cos(e1(1,i))*sqrt(u_R(1,i)^2+u_R(2,i)^2+u_R(3,i)^2)));
b2(1,i) = -asin((u_R(5,i))/(cos(e2(1,i))*sqrt(u_R(4,i)^2+u_R(5,i)^2+u_R(6,i)^2)));
b3(1,i) = asin((u_R(8,i))/(cos(e3(1,i))*sqrt(u_R(7,i)^2+u_R(8,i)^2+u_R(9,i)^2)));
b4(1,i) = asin((u_R(11,i))/(cos(e4(1,i))*sqrt(u_R(10,i)^2+u_R(11,i)^2+u_R(12,i)^2)));

w1(1,i) = (30/pi)*sqrt(sqrt(u_R(1,i)^2+u_R(2,i)^2+u_R(3,i)^2)/(kT));
w2(1,i) = (30/pi)*sqrt(sqrt(u_R(4,i)^2+u_R(5,i)^2+u_R(6,i)^2)/(kT));
w3(1,i) = (30/pi)*sqrt(sqrt(u_R(7,i)^2+u_R(8,i)^2+u_R(9,i)^2)/(kT));
w4(1,i) = (30/pi)*sqrt(sqrt(u_R(10,i)^2+u_R(11,i)^2+u_R(12,i)^2)/(kT));


%     e1(1,i) = asin(u_R(1,i)/(4*sqrt(u_R(1,i)^2+u_R(2,i)^2+u_R(3,i)^2)));
%     e2(1,i) = asin(u_R(4,i)/(4*sqrt(u_R(4,i)^2+u_R(5,i)^2+u_R(6,i)^2)));
%     e3(1,i) = asin(u_R(7,i)/(4*sqrt(u_R(7,i)^2+u_R(8,i)^2+u_R(9,i)^2)));
%     e4(1,i) = asin(u_R(10,i)/(4*sqrt(u_R(10,i)^2+u_R(11,i)^2+u_R(12,i)^2)));
% 
%     b1(1,i) = cos(e1(1,i))*asin(u_R(2,i)/(4*sqrt(u_R(1,i)^2+u_R(2,i)^2+u_R(3,i)^2)));
%     b2(1,i) = cos(e2(1,i))*asin(u_R(5,i)/(4*sqrt(u_R(4,i)^2+u_R(5,i)^2+u_R(6,i)^2)));
%     b3(1,i) = cos(e3(1,i))*asin(u_R(8,i)/(4*sqrt(u_R(7,i)^2+u_R(8,i)^2+u_R(9,i)^2)));
%     b4(1,i) = cos(e4(1,i))*asin(u_R(11,i)/(4*sqrt(u_R(10,i)^2+u_R(11,i)^2+u_R(12,i)^2)));
% 
%     w1(1,i) = (30/pi)*sqrt(sqrt(F(1,i)^2+F(2,i)^2+F(3,i)^2)/(1));
%     w2(1,i) = (30/pi)*sqrt(sqrt(F(4,i)^2+F(5,i)^2+F(6,i)^2)/(1));
%     w3(1,i) = (30/pi)*sqrt(sqrt(F(7,i)^2+F(8,i)^2+F(9,i)^2)/(1));
%     w4(1,i) = (30/pi)*sqrt(sqrt(F(10,i)^2+F(11,i)^2+F(12,i)^2)/(1));

    u(:,i) = [w1(1,i),w2(1,i),w3(1,i),w4(1,i),b1(1,i),b2(1,i),b3(1,i),b4(1,i),e1(1,i),e2(1,i),e3(1,i),e4(1,i)]';

end 

%%%%%%%%%%%% dynamic constraints
% wp = 2;
% bp = 0.3047;
% ep = 0.3047;
% 
% duk = u(:,1) - uold(:,1);
% 
% for i=1:N
% 
%     c1(2*i-1) = -duk(1,1)-wp;
%     c1(2*i) = duk(1,1)-wp;
% 
%     c2(2*i-1) = -duk(2,1)-wp;
%     c2(2*i) = duk(2,1)-wp;
% 
%     c3(2*i-1) = -duk(3,1)-wp;
%     c3(2*i) = duk(3,1)-wp;
% 
%     c4(2*i-1) = -duk(4,1)-wp;
%     c4(2*i) = duk(4,1)-wp;
% 
%     c5(2*i-1) = -duk(5,1)-bp;
%     c5(2*i) = duk(5,1)-bp;
% 
%     c6(2*i-1) = -duk(6,1)-bp;
%     c6(2*i) = duk(6,1)-bp;
% 
%     c7(2*i-1) = -duk(7,1)-bp;
%     c7(2*i) = duk(7,1)-bp;
% 
%     c8(2*i-1) = -duk(8,1)-bp;
%     c8(2*i) = duk(8,1)-bp;
% 
%     c9(2*i-1) = -duk(9,1)-ep;
%     c9(2*i) = duk(9,1)-ep;
% 
%     c10(2*i-1) = -duk(10,1)-ep;
%     c10(2*i) = duk(10,1)-ep;
% 
%     c11(2*i-1) = -duk(11,1)-ep;
%     c11(2*i) = duk(11,1)-ep;
% 
%     c12(2*i-1) = -duk(12,1)-ep;
%     c12(2*i) = duk(12,1)-ep;
% 
%     if i<N
% 
%         duk = u(:,i+1)-u(:,i);
%     end
% 
% 
% end 
% 
%     c = [c1;c2;c3;c4;c5;c6;c7;c8;c9;c10;c11;c12];
%     ceq  = [];


%%%%%%%%%%%% static constraint
c =[];

for i=1:N
    c1 = u(1,i)-1000;
    c2 = -1000-u(1,i);
    c3 = u(2,i)-1000;
    c4 = -1000-u(2,i);
    c5 = u(3,i)-1000;
    c6 = -1000-u(3,i);
    c7 = u(4,i)-1000;
    c8 = -1000-u(4,i);
    c9 = u(5,i)-1.57;
    c10 = -0-u(5,i);
    c11 = u(6,i)-1.57;
    c12 = -0-u(6,i);
    c13 = u(7,i)-1.57;
    c14 = -0-u(7,i);
    c15 = u(8,i)-1.57;
    c16 = -0-u(8,i);
    c17 = u(9,i)-1.57;
    c18 = -0-u(9,i);
    c19 = u(10,i)-1.57;
    c20 = -0-u(10,i);
    c21 = u(11,i)-1.57;
    c22 = -0-u(11,i);
    c23 = u(12,i)-1.57;
    c24 = -0-u(12,i);

    cnew = [c1;c2;c3;c4;c5;c6;c7;c8;c9;c10;c11;c12;c13;c14;c15;c16;c17;c18;c19;c20;c21;c22;c23;c24];
    c = [c cnew];
    ceq  = [];

end 

     
end 
%% UAV mpdel for mpc
function dx = system(t,x,v)

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

%

dx  = zeros(12,1);

dx(1) = (1/Ixx)*((Iyy-Izz)*x(2)*x(3)+v(4)-Ar*x(1));
dx(2) = (1/Iyy)*((Ixx-Izz)*x(1)*x(3)+v(5)-Ar*x(2));
dx(3) = (1/Izz)*((Iyy-Ixx)*x(2)*x(1)+v(6)-Ar*x(3));
dx(4) = x(1) + sin(x(4))*tan(x(5))*x(2) + cos(x(4))*tan(x(5))*x(3);
dx(5) = cos(x(4))*x(2) - sin(x(4))*x(3);
dx(6) = sin(x(4))/cos(x(5))*x(2) + cos(x(4))/cos(x(5))*x(3);
dx(7) = (v(1)*cos(x(6))*cos(x(5)) + v(2)*(cos(x(4))*sin(x(6)) + cos(x(6))*sin(x(4))*sin(x(5))) + v(3)*(sin(x(4))*sin(x(6)) - cos(x(4))*cos(x(6))*sin(x(5))) - Ax*x(7))/m;
dx(8) = (-v(1)*cos(x(5))*sin(x(6)) + v(2)*(cos(x(4))*cos(x(6)) - sin(x(4))*sin(x(6))*sin(x(5))) + v(3)*(cos(x(6))*sin(x(4)) + cos(x(4))*sin(x(6))*sin(x(5))) - Ay*x(8))/m;
dx(9) = (g + (v(1)*sin(x(5)) - v(2)*cos(x(5))*sin(x(4)) + v(3)*cos(x(4))*cos(x(5)) - Az*x(9))/m);
dx(10) = x(7);
dx(11) = x(8);
dx(12) = x(9);

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
% %Rotor Roll 
% beta = [-u(5),-u(6),u(7),u(8)];
% %Rotor Pitch 
% eta = [-u(9),-u(10),u(11),u(12)];

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
%m = 0.68;
g = 9.81;

Ixx=4.856e-03;Iyy=4.856e-03;Izz=8.801e-03;

Ir=3.357e-05;
Ax=0.3; Ay=0.3; Az=0.25; Ar=0.2;
% T1= -k*w(1)^2;
% T2= -k*w(2)^2;
% T3= k*w(3)^2;
% T4= k*w(4)^2;

T1= -k*w(1)^2;
T2= -k*w(2)^2;
T3= k*w(3)^2;
T4= k*w(4)^2;

T = [T1,T2,T3,T4]; % Rotor Thrust

%rotor forces
for i =1:4
    F_R(:,i) = [-T(i)*sin(eta(i));
                +T(i)*cos(eta(i))*sin(beta(i));
                +T(i)*cos(beta(i))*cos(eta(i))];
end

%transform rotor forces to body frame
F_B(:,1) = F_R(:,1);
F_B(:,2) = [-1,0,0;0,-1,0;0,0,1]*F_R(:,2);
F_B(:,3) = [1,0,0;0,-1,0;0,0,-1]*F_R(:,3);
F_B(:,4) = [-1,0,0;0,1,0;0,0,-1]*F_R(:,4);

% F_B(:,1) = F_R(:,1);
% F_B(:,2) = F_R(:,2);
% F_B(:,3) = F_R(:,3);
% F_B(:,4) = F_R(:,4);

%body forces 
FBx = F_B(1,1)+F_B(1,2)+F_B(1,3)+F_B(1,4);
FBy = F_B(2,1)+F_B(2,2)+F_B(2,3)+F_B(2,4);
FBz = F_B(3,1)+F_B(3,2)+F_B(3,3)+F_B(3,4);

% MPhi = l(2)*(-F_R(3,1)+F_R(3,2)+F_R(3,3)-F_R(3,4));
% MThe = l(1)*(F_R(3,1)-F_R(3,2)+F_R(3,3)-F_R(3,4));
% MPsi = kQ*(-F_R(3,1)+F_R(3,2)-F_R(3,3)+F_R(3,4));

MPhi = l(2)*(F_B(3,1)+F_B(3,2)-F_B(3,3)-F_B(3,4))...
    + l(3)*(F_B(2,1)+F_B(2,2)+F_B(2,3)+F_B(2,4));

MThe = l(1)*(-F_B(3,1)+F_B(3,2)-F_B(3,3)+F_B(3,4))...
    + l(3)*(-F_B(2,1)-F_B(2,2)-F_B(2,3)-F_B(2,4));

% MPsi = kQ*(-F_B(3,1)+F_B(3,2)-F_B(3,3)+F_B(3,4))...
%     + l(1)*(F_B(2,1)-F_B(2,2)+F_B(2,3)-F_B(2,4))...
%     + l(2)*(-F_B(1,1)-F_B(1,2)+F_B(1,3)+F_B(1,4));
MPsi = (kQ)*(F_B(3,1)+F_B(3,2)-F_B(3,3)-F_B(3,4))...
    + l(1)*(F_B(2,1)-F_B(2,2)+F_B(2,3)-F_B(2,4))...
    + l(2)*(-F_B(1,1)+F_B(1,2)+F_B(1,3)-F_B(1,4));
%MPsi = kQ*(F_B(3,1)+F_B(3,2)-F_B(3,3)-F_B(3,4));

%Omega = -w(1)+w(2)-w(3)+w(4);

Omega = w(1)-w(2)+w(3)-w(4);

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
