clc; 
clear; 
% close all;

global v_des v_act rotor_Params rotor_Forces
v_des = zeros(6,1);
v_act = zeros(6,1);
rotor_Forces = zeros(12,1);
rotor_Params = zeros(12,1);

t0 = 0;
tf = 60;
x0 = [0,0,0,0,0,0,0,0,0,0,0,0]';

[t1,x1] = ode45(@(t1,x1) clsys(t1,x1,2,t1(end)),[t0 tf],x0);

for ii = 1:length(t1)
XD(ii,:) = trajectory1(t1(ii)).';
end
%%
TV_PosAtt_Animation(XD,rotor_Params,rotor_Forces)
figure()
plot3(x1(1:100:end,10),x1(1:100:end,11),x1(1:100:end,12), 'b:','LineWidth',1.5);
hold on;
plot3(XD(1:100:end,10),XD(1:100:end,11),XD(1:100:end,12), 'r:','LineWidth',1.5);
xlabel('X')
ylabel('Y')
zlabel('Z')
grid on

%%
f1 = figure('Renderer', 'painters', 'Position', [10 10 1600 1000]);
hold on
title('States')
set(0, 'CurrentFigure', f1)
    for i = 1:12
    plotTitle = titlePlot(i);
    subplot(4,3,i)
    title(plotTitle)
    hold on 
    plot(t1(1:10:end),x1(1:10:end,i),'r.-')
    %plot(t2(1:1000:end),x2(1:1000:end,i),'g.-')
    plot(t1(1:10:end),XD(1:10:end,i),'b')
    grid on;
    xlabel('time [s]')
        if i < 4
            ylabel('body rates [rad/s]');
        elseif i<7 && i>3
            ylabel('euler angles [rad]');
        elseif i < 10 && i> 6
            ylabel('velocity [m/s]');
        else
            ylabel('position [m]');
        end
    end
    subplot(4,3,1)
    legend('fbl','lqr','desired','Location','northwest')

f2 = figure('Renderer', 'painters', 'Position', [10 10 1600 1000]);
hold on
title('Virtual Control')
set(0, 'CurrentFigure', f2)
    for i = 1:6 
        subplot(2,3,i)
        title(i)
        hold on 
        plot(v_des(i,100:10:end),'r--')
        plot(v_act(i,100:10:end),'b-.')
        grid on;
    end
    subplot(2,3,1)
    legend('desired','actual','Location','northwest')

function plotTitle = titlePlot(i)
    switch i
        case 1 
            plotTitle = "P";
        case 2 
            plotTitle = "Q";
        case 3 
            plotTitle = "R";
        case 4 
            plotTitle = "Phi";
        case 5 
            plotTitle = "Theta";
        case 6 
            plotTitle = "Psi";
        case 7 
            plotTitle = "U";
        case 8 
            plotTitle = "V";
        case 9 
            plotTitle = "W";
        case 10 
            plotTitle = "X";
        case 11
            plotTitle = "Y";
        case 12 
            plotTitle = "Z";
    end
end
% Feb 10

