close all
clear


qqr = [0,1,0;
    3,1,0;
    4,2,pi/3;
    4,4,pi/3;
    4,6,0];

dt = 0.01;
tv = 0:dt:1;
tqr = tv;
ctr = 1;
for i=1:size(qqr,1)-1
    t0 = (ctr-1)*length(tv);
    ltv = 1:length(tv);
    [qr(t0+ltv,:),dqr(t0+ltv,:)] = jtraj(qqr(i,:), qqr(i+1,:), tv);
    tqr = [tqr,tv+ctr];
    ctr = ctr+1;
end

showtraj(qr,dqr,tqr);

