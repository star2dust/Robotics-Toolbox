function map = map_generator(scale,coord1,width,tile,rad,vqr,dt)

% width = 8.1835;
% scale = 1;
% tile = 1;
% coord1 = [10,12];
% rad = 3;
% vqr = 1;
% dt = 0.01;
coord2 = [coord1(1)+8+ceil(width*10)/10,8];

Voc{1} = [0     0
     8     0
     8     8
     8    16
     8    24
     0    24
     0    16
     0     8
     0     0]*scale+coord1;
Voc{2} = [0     0
     8     0
     8     8
     0     8
     0     0]*scale+coord2;
Voc_min = Voc;
for i = 1:length(Voc)
    xy_min = min(Voc{i});
    xy_max = max(Voc{i});
    Voc_min{i}(Voc{i}==xy_min) = Voc{i}(Voc{i}==xy_min)+tile;
    Voc_min{i}(Voc{i}==xy_max) = Voc{i}(Voc{i}==xy_max)-tile;
end
ws = [-10 40 0 30]*scale;
% for i = 1:length(Voc)
%     plot(Voc{i}(:,1),Voc{i}(:,2),'g'); hold on
%     plot(Voc_min{i}(:,1),Voc_min{i}(:,2),'k');
%     axis(ws)
% end

map.Voc = Voc;
map.Voc_min = Voc_min;
map.ws = ws;


y1 = coord1(2)/2;
x1 = (coord1(1)+coord2(1))/2+4;
qqr = [0,y1,0;
    x1-rad,y1,0;
    x1,y1+rad,pi/2;
    x1,ws(4)-y1-rad,pi/2;
    x1+rad,ws(4)-y1,0;
    ws(2)-y1,ws(4)-y1,0];
qqrdf = diff(qqr);

tauqr = normby(qqrdf(:,1:2),1)/vqr;
tauqr([2,4]) = pi/2*rad/vqr;
tqr = tril(ones(length(tauqr)))*tauqr;

ctr = 1;
xqr = qqr(1,:);
p0 = [0,0;1,0]';
p = SE2(xqr)*p0;

% plot(qqr(:,1),qqr(:,2));
% hgqr = plot(p(1,:),p(2,:));
for t = 0:dt:sum(tauqr)
    [qr(ctr,:),dqr(ctr,:)] = qr_traj(t,tqr,vqr,qqr,qqrdf,tauqr);
%     xqr = xqr + dqr(ctr,:)*dt;
%     p = SE2(xqr)*p0;
%     set(hgqr,'xdata',p(1,:),'ydata',p(2,:));
%     drawnow
    ctr = ctr + 1;
end

map.tqr_via = tqr;
map.qqr_via = qqr;
map.tqr = 0:dt:sum(tauqr);
map.qqr = qr;
map.dqqr = dqr;

save('map.mat','map');

end

function [qr,dqr] = qr_traj(t,tqr,vqr,qqr,qqrdf,tauqr)
if t<tqr(1)
        qr = qqr(1,:)+sign(qqrdf(1,:))*vqr*t;
        dqr = sign(qqrdf(1,:))*vqr;
    elseif t<tqr(2)
        qr = qqr(2,:)+qqrdf(2,:).*[cos(-pi/2+pi/2/tauqr(2)*(t-tqr(1))),...
            1+sin(-pi/2+pi/2/tauqr(2)*(t-tqr(1))),1/tauqr(2)*(t-tqr(1))];
        dqr = qqrdf(2,:).*[-pi/2/tauqr(2)*sin(-pi/2+pi/2/tauqr(2)*(t-tqr(1))),...
            pi/2/tauqr(2)*cos(-pi/2+pi/2/tauqr(2)*(t-tqr(1))),1/tauqr(2)];
    elseif t<tqr(3)
        qr = qqr(3,:)+sign(qqrdf(3,:))*vqr*(t-tqr(2));
        dqr = sign(qqrdf(3,:))*vqr;
    elseif t<tqr(4)
        qr = qqr(4,:)+qqrdf(4,:).*[1+cos(pi-pi/2/tauqr(4)*(t-tqr(3))),...
            sin(pi-pi/2/tauqr(4)*(t-tqr(3))),1/tauqr(4)*(t-tqr(3))];
        dqr = qqrdf(4,:).*[pi/2/tauqr(4)*sin(pi-pi/2/tauqr(4)*(t-tqr(3))),...
            -pi/2/tauqr(4)*cos(pi-pi/2/tauqr(4)*(t-tqr(3))),1/tauqr(4)];
    else
        qr = qqr(5,:)+sign(qqrdf(5,:))*vqr*(t-tqr(4));
        dqr = sign(qqrdf(5,:))*vqr;
end
end

% qr = qqr(1,:);
% for t = 0:0.1:sum(tauqr)
%     [~,dqr] = qr_traj(vqr,t);
%     qr = qr + dqr*0.1;
%     plot(qr(1),qr(2),'ro');
%     drawnow
% end

% % plot data
% load('data.mat')
% robot_lkthick = 2;
% robot_hgsize = 2;
% ws = [-8 8 -8 8];
% plot(qfr(1),qfr(2),'b*');
% for i=1:robot_num
%     hrob_opt(i) = robot_object(i).plot(qa_opt(i,:), qb(i,:),...
%         'workspace', ws, 'dim', length(ws)/2, 'plat',...
%         'hgsize', robot_hgsize, 'lkthick', robot_lkthick);
%     hold on
% end