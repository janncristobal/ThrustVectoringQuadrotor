% trajectory
function XD = trajectory(t,x)

tiltRate = pi/300;
altRate = 0.2;
circRadius = 1;
circRate = pi/30;

xd = circRadius*cos(circRate*t);
yd = circRadius*sin(circRate*t);
zd = altRate*t;
ud = -circRadius*circRate*sin(circRate*t);
vd = circRadius*circRate*cos(circRate*t);
wd = altRate;

phid = xd;
thed = yd;
psid = zd;

pd = ud-wd*sin(thed);
qd = vd*cos(phid)+wd*sin(phid)*cos(thed);
rd = -vd*sin(phid)+wd*cos(phid)*cos(thed);

% tiltRate = pi/300;
% altRate = 0.2;
% circRadius = 1;
% circRate = pi/30;
% 
% xd = 0;
% yd = 0;
% zd = 2;
% ud = 0;
% vd = 0;
% wd = 0;
% 
% phid = 0;
% thed = deg2rad(8);
% psid = 0;
% 
% pd = 0;
% qd = 0;
% rd = 0;

XD = [pd;qd;rd;phid;thed;psid;ud;vd;wd;xd;yd;zd];
end