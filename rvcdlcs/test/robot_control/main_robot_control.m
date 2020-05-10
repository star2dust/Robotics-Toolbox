close all
clear

% cd to current folder
cd(fileparts(mfilename('fullpath')));
savelog('log3.txt', '----------------------------simulation start---------------------------------');

%% offline data

% robot data
tile_w = 1/2;
eps_safe = 0.1;
robot_radius = tile_w-eps_safe;
robot_altitude = tile_w;
robot_height = tile_w;
link_num = 3;
savelog('log3.txt',['link number: ' num2str(link_num)]);

robot_link = ones(1,link_num)*tile_w*0.77;
robot_num = 6;
savelog('log3.txt',['robot number: ' num2str(robot_num)]);
% lidar data
lidar_radius = 4;
lidar_hlim = [0,robot_altitude];
lidar_alim = [-pi,pi];
% map data
map_ind = 3;
savelog('log3.txt',['map index: ' num2str(map_ind)]);
% plot data
robot_lkthick = 2;
robot_hgsize = 2;
ws = [-8 8 -8 8];

% formation displacement (hexagon) 2*sqrt(3)=3.4641=>4.5
hex_scale = 1.5;
Vhex = [-0.5,-sqrt(3)/2;0.5,-sqrt(3)/2;1,0;
    0.5,sqrt(3)/2;-0.5,sqrt(3)/2;-1,0]*tile_w;
pfd = hex_scale*Vhex*3.85; %=> 5/5
thfd = cart2pol(pfd(:,1),pfd(:,2));
% max dis from pe to pb in f frame
pfeb_max = Vhex*2.3; %=>3/5 (1.76-2.3) by link length

% scale range
s_min = (0.86+(hex_scale-1))/hex_scale; s_max = 1; 
savelog('log3.txt',['scale range: [' num2str(s_min) ', ' num2str(s_max) ']']);
% sector space
for i=2:robot_num+1
    I = convhull_(pfd); I = [I(end-1);I]; 
    j = i-1; % => robot ind
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
    % joint range
    qa1_dis(j,:) = norm(qa1_min(j,:)-qa1_max(j,:));
    qae_min(j,:) = -qa1_dis(j,:); qae_max(j,:) = qa1_dis(j,:);
    qa_lim{j} = [qa1_min(j,:),ones(1,link_num-1)*qae_min(j,:);
        qa1_max(j,:),ones(1,link_num-1)*qae_max(j,:)];
end
savelog('log3.txt', 'joint 2 to end range: ');
savelog('log3.txt', ['[' num2str(qae_min') ']']);
savelog('log3.txt', ['[' num2str(qae_max') ']']);
savelog('log3.txt', 'joint 1 range: ');
savelog('log3.txt', ['[' num2str(qa1_min') ']']);
savelog('log3.txt', ['[' num2str(qa1_max') ']']);

% lidar and robot
for i=1:robot_num
    lidar_object(i) = Lidar(lidar_radius,'name',['lidar' num2str(i)],...
        'hlim',lidar_hlim,'alim',lidar_alim,'mapind',map_ind);
    robot_object(i) = PlanarRevolute(robot_link,'name',['rob' num2str(i)],...
        'height',robot_height,'radius',robot_radius,'altitude',...
        robot_altitude,'qlim',qa_lim{i});
end

% max pb in f frame
pfb_max = s_max.*pfd;
% (qfb_max,pfeb_max) => pfe_opt for object
pfe_opt = (SE2(pfb_max)*(-pfeb_max'))';

% s_opt for formation
s_opt = (s_min+s_max)/2; 
% joint range to Aqa and bqe
for i=1:robot_num
    Aqa{i} = [tril(ones(link_num));-tril(ones(link_num))];
    bqa{i} = [kron(ones(link_num,1),qa1_max(i));
        kron(ones(link_num,1),-qa1_min(i))];
end
% calculate a desired qa_opt
import PlanarRevolute.*
for i=1:robot_num
    negmup2 = @(qa) -getMu(robot_link,qa);
    qa_opt(i,:) = fmincon(negmup2,qa_lim{i}(2,:)',Aqa{i},bqa{i},...
        [],[],qa_lim{i}(1,:)',qa_lim{i}(2,:)',...
        @(qa) fkcon(robot_link,qa,pfe_opt(i,:)-s_opt*pfd(i,:)))';
end
qae_opt = qa_opt(:,2:end);

% calculate the desired qfc_opt
phife_opt = sum(qa_opt,2);
qfe_opt = [pfe_opt,phife_opt];
qfc_opt = sum(qfe_opt)/robot_num;
qfb_opt = [s_opt*pfd,zeros(robot_num,1)];

% qrob_opt
qrob_opt = [kron(ones(robot_num,1),s_opt),qfe_opt,qae_opt];

% topology graph
D = [-1,0,0,0,0,1,-1,-1;
    1,-1,0,0,0,0,0,0;
    0,1,-1,0,0,0,0,0;
    0,0,1,-1,0,0,0,1;
    0,0,0,1,-1,0,1,0;
    0,0,0,0,1,-1,0,0;];
L = D*D';

% calculate the max robot team setting
for i=1:robot_num
    negmup2 = @(qa) -getMu(robot_link,qa);
    qa_max(i,:) = fmincon(negmup2,qa_lim{i}(2,:)',Aqa{i},bqa{i},...
        [],[],qa_lim{i}(1,:)',qa_lim{i}(2,:)',...
        @(qa) fkcon(robot_link,qa,pfe_opt(i,:)-s_max*pfd(i,:)))';
end
phife_min = sum(qa_max,2);
qfb_max = [s_max*pfd,zeros(robot_num,1)];

% calculate the min robot team setting
for i=1:robot_num
    negmup2 = @(qa) -getMu(robot_link,qa);
    qa_min(i,:) = fmincon(negmup2,qa_lim{i}(2,:)',Aqa{i},bqa{i},...
        [],[],qa_lim{i}(1,:)',qa_lim{i}(2,:)',...
        @(qa) fkcon(robot_link,qa,pfe_opt(i,:)-s_min*pfd(i,:)))';
end
phife_max = sum(qa_min,2);
qfb_min = [s_min*pfd,zeros(robot_num,1)];

if sum(phife_min-qa1_min<0)
    error('bad phife_min');
end

if sum(phife_max-qa1_max>0)
    error('bad phife_max');
end

%% plot the opt robot team setting
% for i=1:robot_num
%     hrob_opt(i) = robot_object(i).plot(qa_opt(i,:), qfb_opt(i,:),...
%         'workspace', ws, 'dim', length(ws)/2, 'plat',...
%         'hgsize', robot_hgsize, 'lkthick', robot_lkthick);
%     hold on
% end
% 
% % visualize min-opt-max robot team settings
% for j=1:10000
%     for i=1:length(robot_object)
%         robot_object(i).animate(qa_min(i,:), qfb_min(i,:)); 
%     end
%     pause(0.1)
%     for i=1:length(robot_object)
%         robot_object(i).animate(qa_opt(i,:), qfb_opt(i,:));
%     end
%     pause(0.1)
%     for i=1:length(robot_object)
%         robot_object(i).animate(qa_max(i,:), qfb_max(i,:));
%     end
%     pause(0.1)
%     for i=1:length(robot_object)
%         robot_object(i).animate(qa_opt(i,:), qfb_opt(i,:));
%     end
%     pause(0.1)
% end

    
%% initial data for real

% initial start pose for real
qfr = [4,4,0];
savelog('log3.txt', ['initial real f frame: [' num2str(qfr) ']']);

% use max robot team setting as initial
pfre = pfe_opt;
phifre = phife_min;
qfre = [pfre,phifre];

% initial scale
s = kron(ones(robot_num,1),s_max);

% initial end-effector pose for real
qe = q(SE2(qfr)*SE2(qfre));
[~,qe(:,end)] = circinterval(qa1_min+qfr(:,end),qe(:,end));

% initial joint pose 1xm (qa: joints)
qa = qa_max; 
qae = qa(:,2:end);

% initial platform pose for real
qfrb = qfb_max;
qb = q(SE2(qfr)*SE2(qfrb));


%% initial data for estimation/measurement

% initial grasp geometry
% xi setting 
xi = 0.02; % 2cm
xi_phi = 2*pi/180; % 2 degree
ang  = rand(robot_num,1)*2*pi-pi;
qfe = [rand(robot_num,1)*xi.*cos(ang(:,1)),...
    rand(robot_num,1)*xi.*sin(ang(:,1)),...
    rand(robot_num,1)*xi_phi]+qfre;
% circle radius xi => 
% in practical circumscribed polygon as xi/cos(pi/polynum)
xi = max(normby(qfe(:,1:2)-qfre(:,1:2),1));
% initial formation pose 
qf = q(SE2(qe)*SE2(qfe).inv);
pf = qf(:,1:2); phif = qf(:,end);
% test initial condition
PI = eye(robot_num)-(1/robot_num)*...
    ones(robot_num,1)*ones(robot_num,1)';
eps_qf_vec = (PI*qf)';
eps_pf_vec = (PI*pf)';
eps_phif_vec = (PI*phif)';
eps_qf_norm = norm(eps_qf_vec(:));
eps_pf_norm = norm(eps_pf_vec(:));
eps_phif_norm = norm(eps_phif_vec(:));
if eps_phif_norm<=pi/2
    savelog('log3.txt', 'estimated f frame:');
    savelog('log3.txt', ['[' num2str(qf(:,1)') ']']);
    savelog('log3.txt', ['[' num2str(qf(:,2)') ']']);
    savelog('log3.txt', ['[' num2str(qf(:,3)') ']']);
else
    error('bad phif');
end
% tau_s and alpha setting
alpha = 0.5;
eigen = eig(L);
lambda2 = eigen(2);
tau_s = eps_qf_norm^(1-alpha)/...
    ((1-alpha)*(2*lambda2)^((1-alpha)/2));
savelog('log3.txt', ['Lidar frequency: ' num2str(1/tau_s)]);
savelog('log3.txt', ['Lidar time horizon: ' num2str(tau_s)]);
% beta setting
lfe = normby(qfe(:,1:2),1)+xi;
beta = sqrt(sum((lfe.^2+norm(qfc_opt)).^2));
% gamma setting
gamma = 5;
% rho setting
chi = 0;
for i=1:robot_num-1
    for j=i:robot_num
        chi_temp = abs(norm(qfe(i,1:2)-qfe(j,1:2))...
            -norm(qe(i,1:2)-qe(j,1:2)));
        if chi_temp>chi
            chi = chi_temp;
        end
    end
end
rho = 2*(xi+eps_pf_norm+sin(eps_phif_norm)*max(lfe))+chi;
% kappa settings
kappa = 3;
% initial estimation of reference
qrh = qf;

% Qmax constructor
s_lim = [s_min;s_max];
phife_lim = [phife_min,phife_max];
Qmax = Q_constructor(robot_object,qfe,qa_opt,T_min,pfd,xi,s_lim,phife_lim);
if sum(~isinside(qrob_opt,Qmax.A,Qmax.b,Qmax.qlim))
    error('bad qrob_opt');
end

% primal variables
qrob = [s,qfe,qae];
% qrob_min = [s,qe(:,[1,2,end]),qae];

% Lagrangian multipliers
lambda = zeros(robot_num,4);


%% plotter and video

% figure
fig = robot_plotter(robot_object,[],qrob,qrh,qf,pfd,Qmax,...
    val_min,var_min,val_max,var_max);

% write video
video_on = false;
if video_on
    initpath = which('startup.m');
    respath = [initpath(1:end-length('startup.m')) 'Results/'];
    if ~exist(respath,'dir')
        mkdir(respath);
    end
    videoname = [respath 'robot_manipulation_demo_02'];
    writerObj = VideoWriter(videoname);
    open(writerObj);
end


%% simulation and animator

% qrob split
[s,qfe,qae,qa,qb,qe,qc] = qrob_split(robot_object,qrob,qf);

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
Qnow = Qmax;
qdis = normby(qrob-convproj(qrob,Qnow.A,Qnow.b,Qnow.qlim),1);

% zero velocities
dqrh = zeros(size(qrh));
dqf = zeros(size(qf));
dqrob = zeros(size(qrob));
dlambda = zeros(size(lambda));
qr = fig.qqr(1,:);
dqr = zeros(size(qr));
ctr = 0;

% simulation loop - fixed dt
tnow = 0; 
dt = 0.001;
for t = 0:dt:2
    ctr = ctr+1;
    tnow(ctr) = t;
    % choose via point in time 'tnow'
%     qr = interp1(fig.tqr,fig.qqr,tnow(ctr));
%     dqr = interp1(fig.tqr,fig.dqr,tnow(ctr));
    % robot controller - estimation
    dqrh = kron(ones(robot_num,1),dqr)-gamma*(qrh-kron(ones(robot_num,1),qr));
    % robot controller - tracking 
    dqf = dqrh-gamma*(qf-qrh)-D*(sign(D'*qf).*abs(D'*qf).^alpha);
    % robot controller - constraint set and detector
%     if tnow(ctr)-tlid(end)>dtlid
%         qflid = qf + dqf*dtlid;
%         Qnow = Q_updater(robot_object,lidar_object,qrob,qf,qflid,pfd,Qmax,...
%             val_max,var_max);
%         tlid(end+1) = tnow(ctr);
%     end
    % robot controller - optimization
    [dqrob,dlambda] = robot_optimizer(robot_object,qrob,lambda,D,T_min,pfd,Qnow,qrob_opt,kappa,beta);
    % update pose
    qrh = qrh + dqrh*dt;
    qf = qf + dqf*dt;
    qrob = qrob + dqrob*dt;
    lambda = lambda + dlambda*dt;
    % distance
    qdis = normby(qrob-convproj(qrob,Qnow.A,Qnow.b,Qnow.qlim),1);
    % qrob split
    [s,qfe,qae,qe,qa,qb,qc] = qrob_split(robot_object,qrob,qf);
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
    qqdis(ctr+1,:) = qdis;
    % update figure
    fig = robot_animator(fig,robot_object,[],qrob,qrh,qf,...
        Qnow,val_min,var_min,val_max,var_max);
    % video
    if video_on
        f = getframe(gcf);
        f.cdata = imresize(f.cdata,[480,640]);
        writeVideo(writerObj,f);
    end
    % next loop
    if norm([dqrob,dlambda])<10^-3
        break;
    end
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
save('data3.mat');