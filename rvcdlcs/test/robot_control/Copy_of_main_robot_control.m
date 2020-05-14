close all
clear

% cd to current folder
cd(fileparts(mfilename('fullpath')));
logfile = 'log.txt';
savelog(logfile, '----------------------------simulation start---------------------------------','w');

%% offline data

% robot data
tile_w = 1/2;
eps_safe = 0.1;
robot_radius = tile_w-eps_safe;
robot_altitude = tile_w;
robot_height = tile_w;
link_num = 3;
savelog(logfile,['link number: ' num2str(link_num)]);

robot_link = ones(1,link_num)*tile_w*0.77;
robot_num = 6;
savelog(logfile,['robot number: ' num2str(robot_num)]);
% lidar data
lidar_radius = 4;
lidar_hlim = [0,robot_altitude];
lidar_alim = [-pi,pi];


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
savelog(logfile,['scale range: [' num2str([s_min,s_max]) ']']);
% hex size
hex_siz = abs(max(pfd)-min(pfd));
hex_siz_max = s_max*hex_siz;
hex_siz_min = s_min*hex_siz;
savelog(logfile,['max hex size: [' num2str(s_max*hex_siz) ']']);
savelog(logfile,['min hex size: [' num2str(s_min*hex_siz) ']']);
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
savelog(logfile, 'joint 2 to end range: ');
savelog(logfile, num2str(qae_min'));
savelog(logfile, num2str(qae_max') );
savelog(logfile, 'joint 1 range: ');
savelog(logfile, num2str(qa1_min'));
savelog(logfile, num2str(qa1_max'));

% generate map 
map = map_generator;

% lidar and robot
for i=1:robot_num
    lidar_object(i) = Lidar(lidar_radius,'name',['lidar' num2str(i)],...
        'hlim',lidar_hlim,'alim',lidar_alim,'map',map);
    robot_object(i) = PlanarRevolute(robot_link,'name',['rob' num2str(i)],...
        'height',robot_height,'radius',robot_radius,'altitude',...
        robot_altitude,'qlim',qa_lim{i});
end

% max pb in f frame
pfb_max = s_max.*pfd;
% (qfb_max,pfeb_max) => pfe_opt for object
pfe_opt = (SE2(pfb_max)*(-pfeb_max'))';

% s_opt for formation
sigma = 0.7;
s_opt = s_min+(s_max-s_min)*sigma; 
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

savelog(logfile, 'qrob_opt: ');
savelog(logfile, num2str(qrob_opt'));

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

savelog(logfile, 'offline data OK!');

%% plot the opt robot team setting

% % plot data
% robot_lkthick = 2;
% robot_hgsize = 2;
% ws = [-8 8 -8 8];
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
qfr = [3.5,3,0];
savelog(logfile, ['initial qfr: [' num2str(qfr) ']']);

% use max robot team setting as initial
pfre = pfe_opt;
phifre = phife_min;
qfre = [pfre,phifre];
savelog(logfile, 'initial qfre: ');
savelog(logfile, num2str(qfre'))

% initial scale
s = kron(ones(robot_num,1),s_max);

% initial end-effector pose for real
qe = q(SE2(qfr)*SE2(qfre));
[~,qe(:,end)] = circinterval(qa1_min+qfr(:,end),qe(:,end));
savelog(logfile, 'initial qe: ');
savelog(logfile, num2str(qe'))

% initial joint pose 1xm (qa: joints)
qa = qa_max; 
qae = qa(:,2:end);

% initial platform pose for real
qfrb = qfb_max;
qb = q(SE2(qfr)*SE2(qfrb));

save('data.mat');

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
savelog(logfile, ['xi = ' num2str(xi)]);
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
    savelog(logfile, 'initial estimated qf:');
    savelog(logfile, num2str(qf'));
else
    error('bad phif');
end
% tau_s and alpha setting
alpha = 0.5;
savelog(logfile, ['alpha = ' num2str(alpha)]);
eigen = eig(L);
lambda2 = eigen(2);
tau_s = eps_qf_norm^(1-alpha)/...
    ((1-alpha)*(2*lambda2)^((1-alpha)/2));
savelog(logfile, ['Lidar frequency: ' num2str(1/tau_s)]);
savelog(logfile, ['Lidar time horizon: ' num2str(tau_s)]);
% beta setting
lfe = normby(qfe(:,1:2),1)+xi;
beta = sqrt(sum((lfe.^2+norm(qfc_opt)).^2));
savelog(logfile, ['beta = ' num2str(beta)]);
% gamma setting
gamma = 5;
savelog(logfile, ['gamma = ' num2str(gamma)]);
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
savelog(logfile, ['rho = ' num2str(rho)]);
% kappa settings
kappa = 3;
savelog(logfile, ['kappa = ' num2str(kappa)]);

% initial estimation of reference
qrh = qf;
savelog(logfile, 'initial estimated qrh:');
savelog(logfile, num2str(qrh'));

% Qmax constructor
s_lim = [s_min;s_max];
phife_lim = [phife_min,phife_max];
Qmax = Q_constructor(robot_object,qfe,qa_opt,T_min,pfd,xi,s_lim,phife_lim);
if sum(~isinside(qrob_opt,Qmax.A,Qmax.b,Qmax.qlim))
    error('bad qrob_opt');
end

% primal variables
qrob = [s,qfe,qae];
savelog(logfile, 'initial estimated qrob:');
savelog(logfile, num2str(qrob'));

% Lagrangian multipliers
lambda = zeros(robot_num,4);

savelog(logfile, 'initial data OK!');

%% simulation and animator

% figure
fig = robot_plotter(robot_object,lidar_object,qrob,qrh,qf,pfd,Qmax,...
    val_min,var_min,val_max,var_max);

% qrob split
[s,qfe,qae,qa,qb,qe,qc] = qrob_split(robot_object,qrob,qf);

% zero velocities
dqrh = zeros(size(qrh));
dqf = zeros(size(qf));
dqrob = zeros(size(qrob));
dlambda = zeros(size(lambda));
ctr = 0;
lctr = 1;

% simulation loop - fixed dt
Qnow = Qmax;
dt = 0.001;
v = 0.25;
playspeed = 10;
for t = 0:dt:2
    ctr = ctr+1;
    savelog(logfile,['loop ' num2str(ctr) ' (t = ' num2str(t) 's)' ]);
    % get pose trajectory
    [qr,dqr] = qr_traj(v,t);
    % robot controller - estimation
    dqrh = kron(ones(robot_num,1),dqr)-gamma*(qrh-kron(ones(robot_num,1),qr));
    % robot controller - tracking 
    dqf = dqrh-gamma*(qf-qrh)-D*(sign(D'*qf).*abs(D'*qf).^alpha);
    % robot controller - constraint set and detector
    if t>tau_s*lctr
        qfhat = qf + dqf*tau_s;
        Qnow = Q_updater(robot_object,lidar_object,qrob,qf,qfhat,pfd,Qmax,...
            val_max,var_max);
        lctr = lctr+1;
    end
    % robot controller - optimization
    [dqrob,dlambda] = robot_optimizer(robot_object,qrob,lambda,D,T_min,pfd,Qnow,qrob_opt,kappa,beta,logfile);
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
    tt(ctr) = t;
    qqr(ctr,:) = qr;
    qqc(ctr,:) = qc;
    qqrh{ctr} = qrh;
    qqf{ctr} = qf;
    qqrob{ctr} = qrob;
    llambda{ctr} = lambda;
    qqa{ctr} = qa;
    qqb{ctr} = qb;  
    qqe{ctr} = qe;
    qqdis(ctr,:) = qdis;
    % update figure
    if mod(ctr,playspeed)==0
        fig = robot_animator(fig,robot_object,[],qrob,qrh,qf,...
            Qnow,val_min,var_min,val_max,var_max);
    end
    % save frame
    frame(ctr) = getframe(gcf);
    % next loop
    drawnow
end
% write video
video_on = false;
if video_on 
    savevideo('Results/','vdtest',frame,[480,640]);
end

% data to mat
save('data3.mat');