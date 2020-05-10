close all
clear

load('data3.mat');

% tracking error
figure(1); % x
cstr = 'bgmrcy'; 
xqrh = cell2mat_(qqrh,1:6,1);
yqrh = cell2mat_(qqrh,1:6,2);
thqrh = cell2mat_(qqrh,1:6,3);
xqf = cell2mat_(qqf,1:6,1);
yqf = cell2mat_(qqf,1:6,2);
thqf = cell2mat_(qqf,1:6,3);
tnow = [0,tnow];
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
s = cell2mat_(qqrob,1:6,1);
xpfe = cell2mat_(qqrob,1:6,2);
ypfe = cell2mat_(qqrob,1:6,3);
phife = cell2mat_(qqrob,1:6,4);
qa2 = cell2mat_(qqrob,1:6,5);
qa3 = cell2mat_(qqrob,1:6,6);
figure(4); % s
for i=1:6
    plot(tnow,s(i,:),cstr(i)); hold on
end
plot(tnow,Qmax.s_lim(1)*ones(size(tnow)),'k');
plot(tnow,Qmax.s_lim(2)*ones(size(tnow)),'k');
figure(5); % pfe in circle
for i=1:6
    pfe_vec = [xpfe(i,:);ypfe(i,:)];
    pfe_dis = normby(pfe_vec-pfre(i,:)',2);
    plot(tnow,pfe_dis,cstr(i)); hold on
end
plot(tnow,xi/cos(pi/6)*ones(size(tnow)),'k');
figure(6); % inside Q
for i=1:6
    plot(tnow(2:end),qqdis(2:end,i),cstr(i)); hold on
end
% % phife 
% for i=1:6
%     figure;
%     plot(tnow,phife(i,:),cstr(i)); hold on
%     plot(tnow,phife_lim(i,1)*ones(size(tnow)),'k');
%     plot(tnow,phife_lim(i,2)*ones(size(tnow)),'k');
% end
% figure; % qae
% for i=1:6
%     plot(tnow,qa2(i,:),cstr(i)); hold on
%     plot(tnow,qa3(i,:),[cstr(i) '-.']); 
% end
% plot(tnow,Qmax.qae_lim{i}(1,1)*ones(size(tnow)),'k');
% plot(tnow,Qmax.qae_lim{i}(2,1)*ones(size(tnow)),'k');
