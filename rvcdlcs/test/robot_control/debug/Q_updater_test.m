close all
clear

load('debug/Q_updater_test.mat','robot_object','lidar_object','robot_num',...
    'Qmax','pfd','pfe','ze','Aqa','bqa','s_lim','val_min','var_min','val_max','var_max','D');
    
% initial ground pose 1x3 
% (qb: platform, qf: formation, qr: refernce, qrh: estimation of reference)
s_max = s_lim(2); s_min = s_lim(1);
s = ones(robot_num,1)*s_max;
pf = kron(ones(robot_num,1),[10,4]); 
thf = ones(robot_num,1)*0;
qf = [pf,thf]; 
% (s,qf) => qb
qfb = [s.*pfd,zeros(robot_num,1)];
qb = q(SE2(qf)*SE2(qfb));
% qrh randomly given
qrh = qb;

% initial joint pose 1xm (qa: joints)
pe = (SE2(qf)*pfe')';
qa = robot_object.ikine([pe,ze,zeros(robot_num,3)],qb,Aqa,bqa); 
qae = qa(:,2:end);

% initial air pose 1x6 (qe: end-effector, qc: object centroid)
fk = robot_object.fkine(qa,qb);
qe = fk(:,end).toqrpy;
% (qf,qa,qb) => qfe
qfe = toqrpy(SE3.qrpy(qf).inv*SE3.qrpy(qe));
qc = sum(qe)/robot_num;

% primal variables
qrob = [s,qfe,qae];

% Lagrangian multipliers
lambda = zeros(robot_num,3);



% qrob split
s = qrob(:,1); qfe = qrob(:,2:7); qae = qrob(:,8:end);
qe = toqrpy(SE3.qrpy(qf).*SE3.qrpy(qfe));
qa = [qe(:,end)-sum(qae,2),qae];
qb = robot_object.bkine(qa,qe);

% figure
it = [4,5,6];
fig = robot_plotter(robot_object(it),lidar_object(it),qa(it,:),qb(it,:),[],[],...
    qf(it,:),pfd(it,:),Qmax,val_max(it,:),var_max(it,:),val_min(it,:),var_min(it,:));

% test
Qtest = Qmax;
Qtest.A = Qmax.A(it);
Qtest.b = Qmax.b(it);
Qtest.qlim = Qmax.qlim(it);
Qnow = Q_updater(robot_object(it),lidar_object(it),Qtest,qf(it,:),qf(it,:),...
    pfd(it,:),qrob(it,:),val_max(it,:),var_max(it,:));


fig = robot_animator(fig,robot_object(it),lidar_object(it),qa(it,:),qb(it,:),[],[],...
    qf(it,:),Qnow,val_max(it,:),var_max(it,:),val_min(it,:),var_min(it,:));

