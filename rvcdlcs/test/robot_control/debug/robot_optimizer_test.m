close all
clear

load('debug/robot_optimizer_test.mat','robot_object','lidar_object','qa',...
    'qb','qc','qrh','qf','val_min','var_min','pfe_opt','s_opt','T_min',...
    'pfd','rfe','Aqa','bqa','s_lim','D','qfe','s','qae','val_max','var_max');

robot_num = length(robot_object);

% figure
% fig = robot_plotter(robot_object,lidar_object,qa,qb,qc,qrh,qf,pfd,s_lim,...
%     val_min,var_min);

% Qmax constructor
Qmax = Q_constructor(robot_object,pfe_opt,s_opt,T_min,pfd,rfe,Aqa,bqa,s_lim);

% Lagrangian multiplier
lambda = zeros(robot_num,3);

ws = [0 8 0 8];
robot_lkthick = 2;
robot_hgsize = 2;
% qrob_opt 
qa = Qmax.qa_opt;
qb = q(SE2(qf)*SE2(Qmax.qfb_opt));
for i=1:length(robot_object)
    hrob(i) = robot_object(i).plot(qa(i,:), qb(i,:), 'workspace', ws, 'dim', length(ws)/2, 'plat',...
        'hgsize', robot_hgsize, 'lkthick', robot_lkthick); hold on
end
qrob_opt = [Qmax.qrob_opt(:,1:3),qfe(:,3:end-1),Qmax.qrob_opt(:,4:end)];
[dqrob,dlambda] = robot_optimizer(robot_object,Qmax,D,T_min,pfd,qrob_opt,lambda);