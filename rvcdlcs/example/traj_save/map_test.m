function map = map_test(scale,coord1,width,tile,rad,vqr,dt)

test_on = true;
if test_on
    close all
    clear
    
    test_on = true;
    tile = 1;
    scale = 1;
    width = 9;
    coord1 = [10,13];
    rad = 3;
    vqr = [5,5,5,5,5,5,5]';
    dt = 0.01;
end
coord2 = [coord1(1)+8+ceil(width*10)/10,coord1(2)];

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
ws = [-10 40 0 40]*scale;
if test_on
    for i = 1:length(Voc)
        plot(Voc{i}(:,1),Voc{i}(:,2),'g'); hold on
        plot(Voc_min{i}(:,1),Voc_min{i}(:,2),'k');
        axis(ws)
    end
end

map.Voc = Voc;
map.Voc_min = Voc_min;
map.ws = ws;


y1 = coord1(2)/2;
x1 = (coord1(1)+coord2(1))/2+4;
th1 = pi/6;
qqr = [0,y1,0;
    (x1-rad)/3,y1,0;
    x1-rad,y1,0;
    x1,y1+rad,th1;
    x1,2*y1,th1;
    x1,ws(4)-y1-rad,th1;
    x1+rad,ws(4)-y1,th1;
    ws(2)-y1,ws(4)-y1,0];
qqrdf = diff(qqr);
itau= 3;

tauqr = normby(qqrdf,1)./vqr;
tauqr(itau) = pi/2*rad/vqr(itau);
tqr = [0;tril(ones(length(tauqr)))*tauqr];

ctr = 1;
xqr = qqr(1,:);
p0 = [0,0;1,0]';
p = SE2(xqr)*p0;

if test_on
    plot(qqr(:,1),qqr(:,2));
    hgqr = plot(p(1,:),p(2,:));
end
for t = 0:dt:sum(tauqr)
    [qr(ctr,:),dqr(ctr,:)] = qr_traj(t,tqr,vqr,qqr,qqrdf,tauqr,itau);
    if test_on
        xqr = xqr + dqr(ctr,:)*dt;
        p = SE2(xqr)*p0;
        set(hgqr,'xdata',p(1,:),'ydata',p(2,:));
        frame(ctr) = getframe(gcf);
        drawnow
    end
    ctr = ctr + 1;
end

map.tqr_via = tqr;
map.qqr_via = qqr;
map.tqr = 0:dt:sum(tauqr);
map.qqr = qr;
map.dqqr = dqr;

if test_on
    savelog('log_test','log msg test...')
    savevideo('video_test',frame)
    save('map_test','map');
end

end

function [qr,dqr] = qr_traj(t,tqr,vqr,qqr,qqrdf,tauqr,itau)
it = sum(t-tqr>=0);
if sum(it-itau==0)
    [qr,dqr] = rotation(qqr(it,:),qqrdf(it,:),tqr(it),tauqr(it),t);
else
    [qr,dqr] = translation(qqr(it,:),qqrdf(it,:),tqr(it),vqr(it),t);
end
end


function [qr,dqr] = rotation(qqr,qqrdf,tqr,tauqr,t)
qr = qqr+qqrdf.*[cos(-pi/2+pi/2/tauqr*(t-tqr)),...
    1+sin(-pi/2+pi/2/tauqr*(t-tqr)),1/tauqr*(t-tqr)];
dqr = qqrdf.*[-pi/2/tauqr*sin(-pi/2+pi/2/tauqr*(t-tqr)),...
    pi/2/tauqr*cos(-pi/2+pi/2/tauqr*(t-tqr)),1/tauqr];
end

function [qr,dqr] = translation(qqr,qqrdf,tqr,vqr,t)
qr = qqr+qqrdf/norm(qqrdf)*vqr*(t-tqr);
dqr = qqrdf/norm(qqrdf)*vqr;
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