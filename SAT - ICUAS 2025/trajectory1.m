% trajectory - infinity
function XD = trajectory1(t)

% tiltRate = pi/300;
altRate = 0.2;
circRadius = 2;
circRate = pi/30;

xd = circRadius*sin(circRate*t);
yd = cos(0.5*circRadius*circRate*t);
%zd = altRate*t;
zd = 4;
ud = circRadius*circRate*cos(circRate*t);
vd = -0.5*circRadius*circRate*sin(0.5*circRadius*circRate*t);
%wd = altRate;
wd = 0;
phid = 0;
thed = 0;
psid = 0;

pd = 0;
qd = 0;
rd = 0;

XD = [pd;qd;rd;phid;thed;psid;ud;vd;wd;xd;yd;zd];
end