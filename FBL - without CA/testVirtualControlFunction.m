%test virtual control

figure(1)
hold on

for i = 1:100
    w(1:4) = sin((pi/30)*0.1*i); 
    b(1:4) = sin((pi/30)*0.1*i);
    e(1:4) = cos((pi/30)*0.1*i);
    u = [w.';b.';e.'];
    v = virtualControl(u);
    subplot(3,2,1)
    hold on;
    plot(i,v((1)));
    subplot(3,2,2)
    hold on;
    plot(i,v((2)));
    subplot(3,2,3)
    hold on;
    plot(i,v((3)));
    subplot(3,2,4)
    hold on;
    plot(i,v((4)));
    subplot(3,2,5)
    hold on;
    plot(i,v((5)));
    subplot(3,2,6)
    hold on;
    plot(i,v((6)));

end 