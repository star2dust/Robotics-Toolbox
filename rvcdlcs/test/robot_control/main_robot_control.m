close all
clear

% cd to current folder
cd(fileparts(mfilename('fullpath')));

%% offline data

% robot data
tile_w = 1;
eps_safe = 0.1;
robot_radius = tile_w-eps_safe;
robot_altitude = tile_w;
robot_height = tile_w;
link_num = 3;
robot_link = ones(1,link_num)*robot_height*0.77;
robot_num = 6;
% lidar data
lidar_radius = 4;
lidar_hlim = [0,robot_altitude];
lidar_alim = [-pi,pi];
% map data
map_ind = 3;

% formation displacement (hexagon) 2*sqrt(3)=3.4641=>4.5
pfd = [-0.5,-sqrt(3)/2;0.5,-sqrt(3)/2;1,0;
    0.5,sqrt(3)/2;-0.5,sqrt(3)/2;-1,0]*3.85; %=> 5
thfd = cart2pol(pfd(:,1),pfd(:,2));
pfeb_max = [-0.5,-sqrt(3)/2;0.5,-sqrt(3)/2;1,0;
    0.5,sqrt(3)/2;-0.5,sqrt(3)/2;-1,0]*2.3; %=>3 (1.76-2.3)

% sector space
s_min = 0.86; s_max = 1; % pfd==1 => s in 3.0792-3.8490
qae_min = 0; qae_max = pi/2;
for i=2:robot_num+1
    I = convhull_(pfd); I = [I(end-1);I]; j = i-1;
    % l:left, r:right
    val_max(j,:) = s_max*pfd(I(i-1),:)-s_max*pfd(I(i),:);
    var_max(j,:) = s_max*pfd(I(i+1),:)-s_max*pfd(I(i),:);
    val_min(j,:) = s_min*pfd(I(i-1),:)-s_max*pfd(I(i),:);
    var_min(j,:) = s_min*pfd(I(i+1),:)-s_max*pfd(I(i),:);
    T_min{j} = [skew_(var_min(j,:))/norm(skew_(var_min(j,:)));
        -skew_(val_min(j,:))/norm(skew_(val_min(j,:)))];
    thTr_min = atan2(var_min(j,2),var_min(j,1));
    thTl_min = atan2(val_min(j,2),val_min(j,1));
    [qa1_min(j,:),qa1_max(j,:)] = circinterval(thTr_min,thTl_min);
    qa_lim{j} = [qa1_min(j),ones(1,link_num-1)*qae_min;
        qa1_max(j),ones(1,link_num-1)*qae_max];
end

% lidar and robot
for i=1:robot_num
    lidar_object(i) = Lidar(lidar_radius,'name',['lidar' num2str(i)],...
        'hlim',lidar_hlim,'alim',lidar_alim,'mapind',map_ind);
    robot_object(i) = PlanarRevolute(robot_link,'name',['rob' num2str(i)],...
        'height',robot_height,'radius',robot_radius,'altitude',...
        robot_altitude,'qlim',qa_lim{i});
end

% initial/desired pfe 2d
ze = ones(robot_num,1)*(robot_altitude+robot_height);
qfb_max = [s_max.*pfd,zeros(robot_num,1)];
% (qfb_max,pfeb_max) => pfe
pfe = (SE2(qfb_max)*(-pfeb_max'))';


% Qmax constructor
for i=1:robot_num
    Aqa{i} = [ones(1,link_num);-ones(1,link_num)]; 
    bqa{i} = [qa1_max(i);-qa1_min(i)];
end
rfe = 0.1; pfe_opt = pfe; % opt detemined by initial
s_opt = 0.9; s_lim = [s_min;s_max];
Qmax = Q_constructor(robot_object,pfe_opt,s_opt,T_min,pfd,rfe,Aqa,bqa,s_lim);

% topology
D = [-1,0,0,0,0,1,-1,-1;
    1,-1,0,0,0,0,0,0;
    0,1,-1,0,0,0,0,0;
    0,0,1,-1,0,0,0,1;
    0,0,0,1,-1,0,1,0;
    0,0,0,0,1,-1,0,0;];
L = D*D';

    
%% initial data

% initial ground pose 1x3 
% (qb: platform, qf: formation, qr: refernce, qrh: estimation of reference)
s_max = s_lim(2); s_min = s_lim(1);
s = ones(robot_num,1)*s_max;
pf = kron(ones(robot_num,1),[4,4]); 
thf = ones(robot_num,1)*0;
qf = [pf,thf]; 
% (s,qf) => qb
qfb = [s.*pfd,zeros(robot_num,1)];
qb = q(SE2(qf)*SE2(qfb));
% qrh randomly given
qrh = qf;

% initial joint pose 1xm (qa: joints)
pe = (SE2(qf)*pfe')';
qa = robot_object.ikine([pe,ze,zeros(robot_num,3)],qb,Aqa,bqa); 
qae = qa(:,2:end);

% initial air pose 1x6 (qe: end-effector, qc: object centroid)
fk = robot_object.fkine(qa,qb);
qe = fk(:,end).toqrpy;
% (qf,qa,qb) => qfe
qfe = toqrpy(SE3.qrpy(qf).inv*SE3.qrpy(qe));
% range qfe(:,end) within [qa1_min,qa1_max]
[~,qfe(:,end)] = circinterval(qa1_min,qfe(:,end));

% primal variables
qrob = [s,qfe,qae];
qrob_min = [s,qfe(:,[1,2,end]),qae];

% Lagrangian multipliers
lambda = zeros(robot_num,3);


% plot(qf(:,1),qf(:,2),'rd'); hold on
% plot(qb(:,1),qb(:,2),'bo'); 
% plot(pe(:,1),pe(:,2),'b*');
% 
% 
% robot_lkthick = 2;
% robot_hgsize = 2;
% ws = [0 8 0 8];
% vbal_min = (SO2(thf)*val_min')';
% vbar_min = (SO2(thf)*var_min')';
% for i=1:length(robot_object)
%     hrob(i) = robot_object(i).plot(qa(i,:), qb(i,:), 'workspace', ws, 'dim', length(ws)/2, 'plat',...
%         'hgsize', robot_hgsize, 'lkthick', robot_lkthick);
%     quiver(qb(i,1),qb(i,2),vbal_min(i,1),vbal_min(i,2),'color','g');
%     quiver(qb(i,1),qb(i,2),vbar_min(i,1),vbar_min(i,2),'color','g');
% end

%% plotter and video

% figure
fig = robot_plotter(robot_object,lidar_object,qrob,qrh,qf,pfd,Qmax,...
    val_min,var_min,val_max,var_max);

% write video
video_on = true;
if video_on
    initpath = which('startup.m');
    respath = [initpath(1:end-length('startup.m')) 'Results/'];
    if ~exist(respath,'dir')
        mkdir(respath);
    end
    videoname = [respath 'robot_manipulation_demo_01'];
    writerObj = VideoWriter(videoname);
    open(writerObj);
end


%% simulation and animator

% qrob split
[s,pfe,thfe,qae,qa,qb,qe,qc] = qrob_split(robot_object,qrob,qf);

% save intials
qqr = fig.qqr(1,:);
qqc = qc;
qqrh = {qrh};
qqf = {qf};
qqrob = {qrob};
llambda = {lambda};
qqa = {qa};
qqb = {qb};
qqe = {qe};

% zero velocities
dqrh = zeros(size(qrh));
dqf = zeros(size(qf));
dqrob = zeros(size(qrob));
dlambda = zeros(size(lambda));
qr = fig.qqr(1,:);
dqr = zeros(size(qr));
ctr = 0;

% bound and finite-time setting
tnow = 0; Qnow = Qmax;
tlid = 0; dtlid = 1; 
% Qmin constructor
Qmin = Qmax;
for i=1:robot_num
    Qmin.qlim{i}(2,1) = Qmin.qlim{i}(1,1);
end
% rho
for i=1:robot_num
negproj = @(q) -norm(q'-convproj(q',Qmin.A{i},Qmin.b{i},Qmin.qlim{i}));
[~,negrho(i)] = fmincon(negproj,qrob_min(i,:)',Qmax.A{i},Qmax.b{i},[],[],...
    Qmax.qlim{i}(1,:)',Qmax.qlim{i}(2,:)');
end
rho = max(abs(negrho));
% xi
for i=1:robot_num
    m = size(qrob_min,2);
    negdis = @(x) -norm([eye(m) -eye(m)]*x);
    [~,negxi(i)] = fmincon(negdis,[qrob_min(i,:)';qrob_min(i,:)'],...
        kron(eye(2),Qmax.A{i}),[Qmax.b{i};Qmax.b{i}],[],[],...
        [Qmax.qlim{i}(1,:)';Qmax.qlim{i}(1,:)'],...
        [Qmax.qlim{i}(2,:)';Qmax.qlim{i}(2,:)']);
end
xi = max(abs(negxi));
alpha2 = rho/(2*dtlid)+sqrt((1+1/(4*dtlid^2))*rho^2+xi*rho);

% simulation loop - fixed dt
dt = 0.01;
for t = 0:dt:fig.tqr(end)
    ctr = ctr+1;
    tnow(ctr) = t;
    % choose via point in time 'tnow'
    qr = interp1(fig.tqr,fig.qqr,tnow(ctr));
    dqr = interp1(fig.tqr,fig.dqr,tnow(ctr));
    % robot controller - estimation
    gamma = 1;
    dqrh = kron(ones(robot_num,1),dqr)-gamma*(qrh-kron(ones(robot_num,1),qr));
    % robot controller - tracking 
    kappa = 2;
    dqf = dqrh-gamma*(qf-qrh)-kappa*D*(D'*qf);
    % robot controller - constraint set and detector
    if tnow(ctr)-tlid(end)>dtlid
        qflid = qf + dqf*dtlid;
        Qnow = Q_updater(robot_object,lidar_object,qrob,qf,qflid,pfd,Qmax,...
            val_max,var_max);
        tlid(end+1) = tnow(ctr);
    end
    % robot controller - optimization
    [dqrob,dlambda] = robot_optimizer(robot_object,qrob,lambda,D,T_min,pfd,Qnow,alpha2);
    % update pose
    qrh = qrh + dqrh*dt;
    qf = qf + dqf*dt;
    qrob = qrob + dqrob*dt;
    lambda = lambda + dlambda*dt;
    % qrob split
    [s,pfe,thfe,qae,qa,qb,qe,qc] = qrob_split(robot_object,qrob,qf);
    % save data
    qqr(ctr+1,:) = qr;
    qqc(ctr+1,:) = qc;
    qqrh{ctr+1} = qrh;
    qqf{ctr+1} = qf;
    qqrob{ctr+1} = qrob;
    llambda{ctr+1} = lambda;
    qqa{ctr+1} = qa;
    qqb{ctr+1} = qb;  
    qqe{ctr+1} = qe;
    % update figure
    fig = robot_animator(fig,robot_object,lidar_object,qrob,qrh,qf,...
        Qnow,val_min,var_min,val_max,var_max);
    % video
    if video_on
        f = getframe(gcf);
        f.cdata = imresize(f.cdata,[480,640]);
        writeVideo(writerObj,f);
    end
    % next loop
    drawnow
end

% % simulation loop
% playspeed = 1/10; tic; 
% while toc<fig.tqr(end)/playspeed
%     ctr = ctr+1;
%     % choose via point in time 'tnow'
%     qr = interp1(fig.tqr,fig.qqr,tnow(ctr));
%     dqr = interp1(fig.tqr,fig.dqr,tnow(ctr));
%     % robot controller - estimation
%     gamma = 1;
%     dqrh = kron(ones(robot_num,1),dqr)-gamma*(qrh-kron(ones(robot_num,1),qr));
%     % robot controller - tracking 
%     kappa = 2;
%     dqf = dqrh-gamma*(qf-qrh)-kappa*D*(D'*qf);
%     % robot controller - constraint set and detector
%     if tnow(ctr)-tlid(end)>dtlid
%         qflid = qf + dqf*dtlid;
%         Qnow = Q_updater(robot_object,lidar_object,qrob,qf,qflid,pfd,Qmax,...
%             val_max,var_max);
%         tlid(end+1) = tnow(ctr);
%     end
%     % robot controller - optimization
%     [dqrob,dlambda] = robot_optimizer(robot_object,qrob,lambda,D,T_min,pfd,Qnow,alpha2);
%     % calculate dt
%     tnow(ctr+1) = toc*playspeed;
%     dt = tnow(ctr+1) - tnow(ctr);
%     % update pose
%     qrh = qrh + dqrh*dt;
%     qf = qf + dqf*dt;
%     qrob = qrob + dqrob*dt;
%     lambda = lambda + dlambda*dt;
%     % qrob split
%     [s,pfe,thfe,qae,qa,qb,qe,qc] = qrob_split(robot_object,qrob,qf);
%     % save data
%     qqr(ctr+1,:) = qr;
%     qqc(ctr+1,:) = qc;
%     qqrh{ctr+1} = qrh;
%     qqf{ctr+1} = qf;
%     qqrob{ctr+1} = qrob;
%     llambda{ctr+1} = lambda;
%     qqa{ctr+1} = qa;
%     qqb{ctr+1} = qb;  
%     qqe{ctr+1} = qe;
%     % update figure
%     fig = robot_animator(fig,robot_object,lidar_object,qrob,qrh,qf,...
%         Qnow,val_min,var_min,val_max,var_max);
%     % video
%     if video_on
%         f = getframe(gcf);
%         f.cdata = imresize(f.cdata,[480,640]);
%         writeVideo(writerObj,f);
%     end
%     % next loop
%     drawnow
%     toc
% end
% toc
% video to avi
if video_on
    close(writerObj);
end
% data to mat
save('data3.mat','tnow','tlid','qqr','qqrh','qqf','qqrob','llambda','qqa','qqb','qqc','qqe');