function XD = trajectory1(simTime,elements)

circRadius = 2; %2 meter radius 
circRate = pi/30;
stepSize = 100/(elements-1);
phid = zeros(1,elements);
thed = zeros(1,elements);
psid = zeros(1,elements);
pd = zeros(1,elements);
qd = zeros(1,elements);
rd = zeros(1,elements);
ud = zeros(1,elements);
vd = zeros(1,elements);
wd = zeros(1,elements);
xd = zeros(1,elements);
yd = zeros(1,elements);
zd = zeros(1,elements);

    for i = 1:elements
        % 0-10s
        if i <= (10/stepSize)+1
        xd(1,i) = 0;
        yd(1,i) = 0; 
        zd(1,i) = 0.2*simTime(i);
        ud(1,i) = 0;
        vd(1,i) = 0; 
        wd(1,i) = 0.2;
        %disp('yes 0 - 10')
        % 10-20s
        elseif i <= (20/stepSize)+1 && i > (10/stepSize)+1
        xd(1,i) = 0.2*(simTime(i)-10);
        yd(1,i) = 0; 
        zd(1,i) = 2;
        ud(1,i) = 0.2;
        vd(1,i) = 0; 
        wd(1,i) = 0;
        %disp('yes 10 - 20')
        % 20-80ss
        elseif i <= (80/stepSize)+1 && i > (20/stepSize)+1
        xd(1,i) = circRadius*cos(circRate*(simTime(i)-20));
        yd(1,i) = circRadius*sin(circRate*(simTime(i)-20));
        zd(1,i) = 2;
        ud(1,i) = -circRate*circRadius*sin(circRate*(simTime(i)-20));
        vd(1,i) = circRate*circRadius*cos(circRate*(simTime(i)-20));
        wd(1,i) = 0;
        %disp('yes 20 - 80')
        % 80-90s
        elseif i <= (90/stepSize)+1 && i>(80/stepSize)+1
        xd(1,i) = 2 - 0.2*(simTime(i)-80);
        yd(1,i) = 0; 
        zd(1,i) = 2;
        ud(1,i) = 0 - 0.2;
        vd(1,i) = 0; 
        wd(1,i) = 0;
        %disp('yes 80 - 90')
        % 90-100s
        elseif i<=(100/stepSize)+1 && i>(90/stepSize)+1
        xd(1,i) = 0;
        yd(1,i) = 0; 
        zd(1,i) = 2-0.2*(simTime(i)-90);
        ud(1,i) = 0;
        vd(1,i) = 0; 
        wd(1,i) = 0-0.2;
        %disp('yes 90 - 100')
        end
    end

XD = [pd;qd;rd;phid;thed;psid;ud;vd;wd;xd;yd;zd];

end