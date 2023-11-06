% controller 
function [F,M] = lqr_controller(x,xd,K_pos1,K_pos2,K_att1,K_att2)
%states 
%Position       
X = x(10);Y = x(11);Z = x(12);
Xd = xd(10);Yd = xd(11);Zd = xd(12);
%Velocity
U = x(7);V = x(8);W = x(9);
Ud = xd(7);Vd = xd(8);Wd = xd(9);
%Euler Angles
Phi = x(4);The = x(5);Psi = x(6);
Phid = xd(4);Thed = xd(5);Psid = xd(6);
%Body Rates
P = x(1);Q = x(2);R = x(3);
Pd = xd(1);Qd = xd(2);Rd = xd(3);


%position control 
%pos_d = [0;0;0;0;0;0];
pos_d = [Xd;Yd;Zd;Ud;Vd;Wd];
pos = [X;Y;Z;U;V;W];
pos_err = pos_d - pos;
%pos_err = pos - pos_d;

F = 10.*K_pos1*pos_err + K_pos2*pos_d;

%attitude control
%att_d = [0;0;0;0;0;0];
att_d = [Phid;Thed;Psid;Pd;Qd;Rd];
att = [Phi;The;Psi;P;Q;R];
att_err = att_d-att;
M = 10.*K_att1*att_err + K_att2*att_d;

end