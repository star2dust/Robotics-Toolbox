close all
clear

load('path_gap_example.mat');
test_rloc = 1;
if test_rloc==1
    [qt,~,~,tqt] = calctraj(rloc1,0.5*ones(size(rloc1(1,:))),0.1,2);
else
    [qt,~,~,tqt] = calctraj(rloc2,0.5*ones(size(rloc2(1,:))),0.1,2);
end
qrot0 = [0,0,0,0];
qrot1 = [1,1,2,1];
dqrot = (qrot1-qrot0)/(length(tqt)-1);
qr = zeros(length(tqt),length(qrot0));
for i=1:length(qrot0)
    qr(:,i) = (qrot0(i):dqrot(i):qrot1(i))';
end
m = 3; link = ones(m,1)*0.8; edge = [1,.8,.5]; mount = [0.2,0.2,edge(3)/2+1];
mpr = MobilePlanarRevolute(edge,link,mount,'name','rob1','type','elbowup');
qt(:,3) = ones(size(qt(:,3)))*edge(3)/2;

figure;
for i=1:length(obs)
   obs(i).plot(qobs(i,:),'workspace', [0 20 0 20 0 20],'facecolor','k','facealpha',0.5);
   hold on; 
end
mpr.plot([qt(1,1:2),qr(1,:)],'frame');
plot3(qt(:,1),qt(:,2),qt(:,3),'r-','linewidth',2);
view(3); xlabel('x');ylabel('y');zlabel('z'); rotate3d on;
hold off

tic;
playspeed = 2;
while toc<tqt(end)/playspeed
    tnow = toc*playspeed;
    % choose via point in time 'tnow'
    qqt = interp1(tqt,qt,tnow); 
    qqr = interp1(tqt,qr,tnow); 
    % update object
    mpr.animate([qqt(1:2),qqr]);
    drawnow
end