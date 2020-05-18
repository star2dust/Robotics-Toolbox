close all
clear

% cd to current folder
cd(fileparts(mfilename('fullpath')));
logfile = 'log.txt';
savelog(logfile, '----------------------------simulation start---------------------------------','w');

%% offline data

% robot data
tile_w = 1;
eps_safe = 0.1;
robot_radius = tile_w-eps_safe;
robot_altitude = tile_w;
robot_height = tile_w;
link_num = 3;
savelog(logfile,['link number: ' num2str(link_num)]);

link_length = 1; % => 1/5
robot_link = tile_w*[1 2 1]*link_length;
robot_num = 6;
savelog(logfile,['robot number: ' num2str(robot_num)]);
% lidar data
lidar_radius = 10;
lidar_hlim = [0,robot_altitude];
lidar_alim = [-pi,pi];


% formation displacement (hexagon) 2*sqrt(3)=3.4641=>4.5
Vhex = [-0.5,-sqrt(3)/2;0.5,-sqrt(3)/2;1,0;
    0.5,sqrt(3)/2;-0.5,sqrt(3)/2;-1,0];
hex_scale = 6*tile_w; 
pfd = Vhex*link_length; % => 6/6
thfd = cart2pol(pfd(:,1),pfd(:,2));
% max dis from pe to pb in f frame
dis_scale = 3.8*tile_w;
pfeb_max = Vhex*link_length*dis_scale; %=> 3.8/6 (<4/6)


% s range (0.8441)
s_min = (3.8*cos(0.8441)+2.2)/6*hex_scale; % 0.7875
s_max = 1*hex_scale;
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
    % qa sign
    if j<=3
        qa_min(j,:) = [qa1_min(j,:),qae_max(j,:),qae_min(j,:)];
    else
        qa_min(j,:) = [qa1_max(j,:),qae_min(j,:),qae_max(j,:)];
    end
end
savelog(logfile, 'joint 2 to end range: ');
savelog(logfile, num2str(qae_min'));
savelog(logfile, num2str(qae_max') );
savelog(logfile, 'joint 1 range: ');
savelog(logfile, num2str(qa1_min'));
savelog(logfile, num2str(qa1_max'));


% generate map 
obs_scale = 1;
hex_width = sum(hex_siz_min)/2;
obs_coord = [10,13];
qr_rad = 3;
qr_vel = [1,5,1,1,5,1,5]';
dt = 0.01;
if ~exist('map.mat','file')
    map_generator(obs_scale,obs_coord,hex_width,tile_w,qr_rad,qr_vel,dt);
end
load('map.mat','map');

% lidar and robot
for i=1:robot_num
    lidar_object(i) = Lidar(lidar_radius,'name',['lidar' num2str(i)],...
        'hlim',lidar_hlim,'alim',lidar_alim);
    robot_object(i) = PlanarRevolute(robot_link,'name',['rob' num2str(i)],...
        'height',robot_height,'radius',robot_radius,'altitude',...
        robot_altitude,'qlim',qa_lim{i});
end

% max pb in f frame
pfb_max = s_max.*pfd;
% (qfb_max,pfeb_max) => pfe_opt for object
pfe_opt = (SE2(pfb_max)*(-pfeb_max'))';


% joint range to Aqa and bqe
for i=1:robot_num
    Aqa{i} = [tril(ones(link_num));-tril(ones(link_num))];
    bqa{i} = [kron(ones(link_num,1),qa1_max(i));
        kron(ones(link_num,1),-qa1_min(i))];
end

% s_opt for formation
sigma = 0.6;
s_opt = s_min+(s_max-s_min)*sigma; 
% calculate a desired qa_opt
import PlanarRevolute.*
for i=1:robot_num
    negmup2 = @(qa) -getMu(robot_link,qa);
    qa_opt(i,:) = fmincon(negmup2,qa_min(i,:)',Aqa{i},bqa{i},...
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
    qa_max(i,:) = fmincon(negmup2,qa_min(i,:)',Aqa{i},bqa{i},...
        [],[],qa_lim{i}(1,:)',qa_lim{i}(2,:)',...
        @(qa) fkcon(robot_link,qa,pfe_opt(i,:)-s_max*pfd(i,:)))';
end
phife_max = sum(qa_max,2);
qfb_max = [s_max*pfd,zeros(robot_num,1)];

% calculate the min robot team setting
phife_min = sum(qa_min,2);
qfb_min = [s_min*pfd,zeros(robot_num,1)];

% qrob_min and qrob_max
qfe_min = [pfe_opt,phife_min];
qfe_max = [pfe_opt,phife_max];
qfc_min = sum(qfe_min)/robot_num;
qfc_max = sum(qfe_max)/robot_num;
qrob_min = [kron(ones(robot_num,1),s_min),qfe_opt,qa_min(:,2:end)];
qrob_max = [kron(ones(robot_num,1),s_max),qfe_opt,qa_max(:,2:end)];

if sum(~(phife_min-qa1_min>-10^-4&phife_min-qa1_max<10^-4))
    error('bad phife_min');
end

if sum(~(phife_max-qa1_min>-10^-4&phife_max-qa1_max<10^-4))
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
iqr = 1;
qfr = map.qqr_via(iqr,:);
savelog(logfile, ['initial qfr: [' num2str(qfr) ']']);

% use max robot team setting as initial
pfre = pfe_opt;
phifre = phife_max;
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

save('data_initial.mat');

%% initial data for estimation/measurement

% initial grasp geometry
% xi setting 
xi = 0.1; % 1cm
xi_phi = 0.2*pi/180; % 2 degree
ang  = rand(robot_num,1)*2*pi-pi;
qfe = [rand(robot_num,1)*xi.*cos(ang(:,1)),...
    rand(robot_num,1)*xi.*sin(ang(:,1)),...
    rand(robot_num,1)*xi_phi]+qfre;
if ~(sum(qfe-qfe_opt>0)<=4&sum(qfe-qfe_opt>0)<=2)
     error('bad qfe');
end
% circle radius xi => 
% in practical circumscribed polygon as xi/cos(pi/polynum)
% xi = max(normby(qfe(:,1:2)-qfre(:,1:2),1));
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
alpha = 0.3;
savelog(logfile, ['alpha = ' num2str(alpha)]);
eigen = eig(L);
lambda2 = eigen(2);
tau_s = eps_qf_norm^(1-alpha)/...
    ((1-alpha)*(2*lambda2)^((1-alpha)/2));
savelog(logfile, ['Lidar frequency: ' num2str(1/tau_s)]);
savelog(logfile, ['Lidar time horizon: ' num2str(tau_s)]);
% beta setting
beta = sqrt(sum(norm(qfe-qfc_opt).^2));
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
lfe = normby(qfe(:,1:2),1)+xi;
rho = 2*(xi+eps_pf_norm+sin(eps_phif_norm)*max(lfe))+chi;
savelog(logfile, ['rho = ' num2str(rho)]);

% initial estimation of reference
qrh = qf;
savelog(logfile, 'initial estimated qrh:');
savelog(logfile, num2str(qrh'));

% Qmax constructor
s_lim = [s_min;s_max];
Qmax = Q_constructor(robot_object,qfe,qa_opt,T_min,pfd,xi,s_lim);
if sum(~isinside(qrob_opt,Qmax.A,Qmax.b,Qmax.qlim))
    error('bad qrob_opt');
end

% Qmin
Qmin = Qmax; 
for i=1:robot_num
    Qmin.qlim{i}(:,1) = [s_min;s_min+0.05];
    Qmin.slim = [s_min;s_min+0.05];
end

% kappa settings
Q = [eye(length(qrob_opt)),-eye(length(qrob_opt));
    -eye(length(qrob_opt)),eye(length(qrob_opt))];
for i=1:robot_num
    fx = @(x) -x'*Q*x;
    x0 = [qrob_opt(i,:),-qrob_opt(i,:)]';
    A = blkdiag(Qmin.A{i},Qmin.A{i});
    b = [Qmin.b{i};Qmin.b{i}];
    qlim = [Qmin.qlim{i},Qmin.qlim{i}];
    [xopt,fxopt] = fmincon(fx,x0,A,b,...
        [],[],qlim(1,:)',qlim(2,:)');
    varrho(i) = sqrt(-fxopt);
    zeta(i) = varrho(i)/(2*tau_s)+sqrt((2+1/(4*tau_s^2))*varrho(i)^2);
end
kappa = max(max(varrho)/2,max(zeta));
savelog(logfile, ['kappa = ' num2str(kappa)]);

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

set(gcf,'position',[0.05 0 0.8 0.8])

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
tlid = [0,tau_s/2];
playspeed = 1;
video_on = false;
for t = map.tqr_via(iqr):dt:map.tqr_via(iqr+1)%map.tqr(end)
    ctr = ctr+1;
    savelog(logfile,['loop ' num2str(ctr) ' (t = ' num2str(t) 's)' ]);
    % get pose trajectory
    qr = interp1(map.tqr,map.qqr,t); 
    dqr = interp1(map.tqr,map.dqqr,t);
    % robot controller - estimation
    dqrh = kron(ones(robot_num,1),dqr)-gamma*(qrh-kron(ones(robot_num,1),qr));
    % robot controller - tracking 
    if abs(D'*qf)<10^-2
        alpha = 1;
        savelog(logfile,'qf stabilized');
    end
    dqf = dqrh-gamma*(qf-qrh)-D*(sign(D'*qf).*abs(D'*qf).^alpha);
    % robot controller - constraint set and detector
    if t>=tau_s*lctr
        tlid = tau_s*lctr+[0,tau_s/2];
        qfhat = qf + dqf*tau_s;
        Qnow = Q_updater(robot_object,lidar_object,qrob,qf,qfhat,pfd,Qmax,...
            val_max,var_max);
        lctr = floor(t/tau_s)+1;
    end
    % robot controller - optimization
    [dqrob,dlambda] = robot_optimizer(robot_object,qrob,lambda,D,T_min,pfd,Qnow,qrob_opt,kappa,beta,logfile);
    % update pose
    qrh = qrh + dqrh*dt;
    qf = qf + dqf*dt;
    qrob = qrob + dqrob*dt;
    lambda = lambda + dlambda*dt;
    % save log
    savelog(logfile, 'current qrh:');
    savelog(logfile, num2str(qrh'));
    savelog(logfile, 'current qf:');
    savelog(logfile, num2str(qf'));
    savelog(logfile, 'current qrob:');
    savelog(logfile, num2str(qrob'));
    savelog(logfile, 'current dqrh:');
    savelog(logfile, num2str(dqrh'));
    savelog(logfile, 'current dqf:');
    savelog(logfile, num2str(dqf'));
    savelog(logfile, 'current dqrob:');
    savelog(logfile, num2str(dqrob'));
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
        fig = robot_animator(fig,robot_object,lidar_object,qrob,qrh,qf,...
            Qnow,val_min,var_min,val_max,var_max);
        if t>=tlid(1)&&t<=tlid(2)
            for j=1:robot_num
                handles = fig.hlid(j).group;
                for i=1:length(handles.Children)
                    if handles.UserData
                        tag = get(handles.Children(i),'Tag');
                        if strcmp(tag(1:end-1), [lidar_object(j).name '-detect'])
                            set(handles.Children(i),'Visible', 'off');
                        end
                    end
                end
            end
        end
        % save frame
        if video_on
            frame(ctr) = getframe(gcf);
        end
    end
    % next loop
    drawnow
end
% write video
if video_on 
    savevideo('Results/','robot_manipulation_demo_05',frame);
end

% data to mat
save('data_final.mat');