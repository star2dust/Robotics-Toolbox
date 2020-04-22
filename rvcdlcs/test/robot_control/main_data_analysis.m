close all
clear

load('data3.mat','tnow','tlid','qqr','qqrh','qqf','qqrob','llambda','qqa','qqb','qqc','qqe');

% tracking error
figure(1); % x
cstr = 'bgmrcy'; 
xqrh = cell2mat_(qqrh,1:6,1);
yqrh = cell2mat_(qqrh,1:6,2);
thqrh = cell2mat_(qqrh,1:6,3);
xqf = cell2mat_(qqf,1:6,1);
yqf = cell2mat_(qqf,1:6,2);
thqf = cell2mat_(qqf,1:6,3);
for i=1:6
    plot(tnow,xqrh(i,:)-qqr(:,1)',cstr(i)); hold on
    plot(tnow,xqf(i,:)-qqr(:,1)',[cstr(i) '-.']);
end
figure(2); % y
for i=1:6
    plot(tnow,yqrh(i,:)-qqr(:,2)',cstr(i)); hold on
    plot(tnow,yqf(i,:)-qqr(:,2)',[cstr(i) '-.']);
end
figure(3); % th
for i=1:6
    plot(tnow,thqrh(i,:)-qqr(:,3)',cstr(i)); hold on
    plot(tnow,thqf(i,:)-qqr(:,3)',[cstr(i) '-.']);
end