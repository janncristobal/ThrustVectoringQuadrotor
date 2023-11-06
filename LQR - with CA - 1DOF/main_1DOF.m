%main1DOF TV Drone 

clc;
clear;
close all;

t0 = 0;
%tf = 0.055;
tf = 1;
x0 = [0,0.2249,0.0199,1.0000,0,0,0,0.1047,0.2,1,0,0]';
%x0 = [0,0,0,0,0,0,0,0,0,0,0,0]';
[t,x] = ode45(@(t,x) clsys(t,x),[t0 tf],x0);

for ii = 1:length(t)
XD(ii,:) = trajectory(t(ii),x(ii,:)).';
end
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
    plot(t(1:10:end),x(1:10:end,i),'r.-')
    plot(t(1:10:end),XD(1:10:end,i),'b')
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
    legend('actual','desired','Location','northwest')

% f2 = figure('Renderer', 'painters', 'Position', [10 10 1600 1000]);
% hold on
% title('Virtual Control')
% set(0, 'CurrentFigure', f2)
%     for i = 1:6 
%         subplot(2,3,i)
%         title(i)
%         hold on 
%         plot(v_des(i,100:10:end),'r--')
%         plot(v_act(i,100:10:end),'b-.')
%         grid on;
%     end
%     subplot(2,3,1)
%     legend('desired','actual','Location','northwest')

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