close all
clear

load('../data/data_final.mat');

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
    plot(tt,xqrh(i,:)-qqr(:,1)',cstr(i)); hold on
    plot(tt,xqf(i,:)-qqr(:,1)',[cstr(i) '-.']);
end
figure(2); % y
for i=1:6
    plot(tt,yqrh(i,:)-qqr(:,2)',cstr(i)); hold on
    plot(tt,yqf(i,:)-qqr(:,2)',[cstr(i) '-.']);
end
figure(3); % th
for i=1:6
    plot(tt,thqrh(i,:)-qqr(:,3)',cstr(i)); hold on
    plot(tt,thqf(i,:)-qqr(:,3)',[cstr(i) '-.']);
end
s = cell2mat_(qqrob,1:6,1);
xpfe = cell2mat_(qqrob,1:6,2);
ypfe = cell2mat_(qqrob,1:6,3);
phife = cell2mat_(qqrob,1:6,4);
qa2 = cell2mat_(qqrob,1:6,5);
qa3 = cell2mat_(qqrob,1:6,6);
figure(4); % s
for i=1:6
    plot(tt,s(i,:),cstr(i)); hold on
end
plot(tt,Qmax.s_lim(1)*ones(size(tt)),'k');
plot(tt,Qmax.s_lim(2)*ones(size(tt)),'k');
figure(5); % pfe in circle
for i=1:6
    pfe_vec = [xpfe(i,:);ypfe(i,:)];
    pfe_dis = normby(pfe_vec-pfre(i,:)',2);
    plot(tt,pfe_dis,cstr(i)); hold on
end
plot(tt,xi/cos(pi/6)*ones(size(tt)),'k');
figure(6); % inside Q
for i=1:6
    plot(tt(2:end),qqdis(2:end,i),cstr(i)); hold on
end
% % phife 
% for i=1:6
%     figure;
%     plot(tt,phife(i,:),cstr(i)); hold on
%     plot(tt,phife_lim(i,1)*ones(size(tt)),'k');
%     plot(tt,phife_lim(i,2)*ones(size(tt)),'k');
% end
% figure; % qae
% for i=1:6
%     plot(tt,qa2(i,:),cstr(i)); hold on
%     plot(tt,qa3(i,:),[cstr(i) '-.']); 
% end
% plot(tt,Qmax.qae_lim{i}(1,1)*ones(size(tt)),'k');
% plot(tt,Qmax.qae_lim{i}(2,1)*ones(size(tt)),'k');
