close all
clear


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
pfd = [-0.5,-sqrt(3)/2;0.5,-sqrt(3)/2;1,0;0.5,sqrt(3)/2;-0.5,sqrt(3)/2;-1,0]*3.85; %=> 5
thfd = cart2pol(pfd(:,1),pfd(:,2));
peb = [-0.5,-sqrt(3)/2;0.5,-sqrt(3)/2;1,0;0.5,sqrt(3)/2;-0.5,sqrt(3)/2;-1,0]*2.3; %=>3 (1.76-2.3)
s_min = 0.86; s_max = 1; % pfd 3.0792-3.8490
qae_min = 0; qae_max = pi/2;
for i=2:robot_num+1
    I = convhull_(pfd); I = [I(end-1);I]; j = i-1;
    % l:left, r:right
    val_min(j,:) = s_min*pfd(I(i-1),:)-s_max*pfd(I(i),:);
    var_min(j,:) = s_min*pfd(I(i+1),:)-s_max*pfd(I(i),:);
    val_max(j,:) = s_max*pfd(I(i-1),:)-s_min*pfd(I(i),:);
    var_max(j,:) = s_max*pfd(I(i+1),:)-s_min*pfd(I(i),:);
    T_min{j} = [skew_(val_min(j,:))/norm(skew_(val_min(j,:)));
        skew_(var_min(j,:))/norm(skew_(var_min(j,:)))];
    T_max{j} = [skew_(val_max(j,:))/norm(skew_(val_max(j,:)));
        skew_(var_max(j,:))/norm(skew_(var_max(j,:)))];
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
        'height',robot_height,'radius',robot_radius,'altitude',robot_altitude,'qlim',qa_lim{i});
end

% initial ground pose 1x3 
% (qb: platform, qf: formation, qr: refernce, qrh: estimation of reference)
s = ones(robot_num,1)*s_max;
pf = kron(ones(robot_num,1),[10,4]); qf = [pf,zeros(robot_num,1)]; 
pb = pf+s.*pfd;%pb = [1,1; 7,1; 7,7; 1,7];% pb = pc+s_min*pcd;
qb = [pb,zeros(robot_num,1)];
qrh = qb;

% initial air pose 1x6 (qa: joints, qc: object centroid, qe: end-effector)
ze = ones(robot_num,1)*(robot_altitude+robot_height);
pe = pf+s_max*pfd-peb;%pe = [3,3; 5,3; 5,5; 3,5];%
for i=1:robot_num
    Aqa{i} = [ones(1,link_num);-ones(1,link_num)]; 
    bqa{i} = [qa1_max(i);-qa1_min(i)];
end
qa = robot_object.ikine([pe,ze,zeros(robot_num,3)],qb,Aqa,bqa); 
qae = qa(:,2:end);
fk = robot_object.fkine(qa,qb);
qe = fk(:,end).toqrpy;
qfe = toqrpy(SE3.qrpy(qf).inv*SE3.qrpy(qe));
qc = sum(qe)/robot_num;

% figure
s_lim = [s_min;s_max];
fig = robot_plotter(robot_object,lidar_object,qa,qb,qc,qrh,qf,pfd,s_lim,val_min,var_min);

% Qmax constructor
rfe = 0.1; pfe_opt = qfe(:,1:2); s_opt = 0.9;
Qmax = Q_constructor(robot_object,pfe_opt,s_opt,T_min,pfd,rfe,Aqa,bqa,s_lim);

% Lagrangian multiplier
lambda = zeros(robot_num,3);


% topology
D = [-1,0,0,0,0,1,-1,-1;
    1,-1,0,0,0,0,0,0;
    0,1,-1,0,0,0,0,0;
    0,0,1,-1,0,0,0,1;
    0,0,0,1,-1,0,1,0;
    0,0,0,0,1,-1,0,0;];
L = D*D';



% write video
video_on = false;
if video_on
    initpath = which('startup.m');
    respath = [initpath(1:end-length('startup.m')) 'Results/'];
    if ~exist(respath,'dir')
        mkdir(respath);
    end
    videoname = [respath 'story_demo'];
    writerObj = VideoWriter(videoname);
    open(writerObj);
end

tic; t0 = 0;
playspeed = 1;
while toc<fig.tqr(end)/playspeed
    tnow = toc*playspeed;
    dt = tnow - t0;
    t0 = tnow;
    % choose via point in time 'tnow'
    qr = interp1(fig.tqr,fig.qqr,tnow);
    dqr = interp1(fig.tqr,fig.dqr,tnow);
    % robot controller - estimation
    gamma = 1;
    dqrh = kron(ones(robot_num,1),dqr)-gamma*(qrh-kron(ones(robot_num,1),qr));
    % robot controller - tracking 
    kappa = 2;
    dqf = dqrh-gamma*(qf-qrh)-kappa*D*(D'*qf);
    % robot controller - constraint set and detector
    qrob = [s,qfe,qae];
    qf_next = qf + dqf*dt;
    save('debug/Q_updater_test.mat','robot_object','lidar_object','Qmax','qf',...
        'qf_next','pfd','qrob','s_lim','val_min','var_min','val_max','var_max');
    Qnow = Q_updater(robot_object,lidar_object,Qmax,qf,qf_next,pfd,qrob,val_max,var_max);
    % robot controller - optimization
    [dqrob,dlambda] = robot_optimizer(robot_object,Qnow,D,T_min,pfd,qrob,lambda);
    % update pose
    qrh = qrh + dqrh*dt;
    qf = qf + dqf*dt;
    qrob = qrob + dqrob*dt;
    lambda = lambda + dlambda*dt;
    s = qrob(:,1); qfe = qrob(:,2:7); qae = qrob(:,8:end);
    qe = toqrpy(SE3.qrpy(qf).*SE3.qrpy(qfe));
    qa = [qe(:,end)-sum(qae,2),qae];
    qb = robot_object.bkine(qa,qe);
    qc = sum(qe)/robot_num;
    % update figure
    fig = robot_animator(fig,robot_object,lidar_object,qa,qb,qc,qrh,qf);
    % video
    if video_on
        f = getframe(gcf);
        f.cdata = imresize(f.cdata,[480,640]);
        writeVideo(writerObj,f);
    end
    drawnow
end
toc
if video_on
    close(writerObj);
end