function [dqrob,dlambda] = robot_optimizer(robot,qrob,lambda,D,T_min,pfd,Qnow,qrob_opt,kappa,beta,logfile)

% qrob lambda split
[s,qfe,qae] = qrob_split(robot,qrob);
[s_opt,qfe_opt,qae_opt,~,~,~,qfc_opt] = qrob_split(robot,qrob_opt,zeros(1,3));
qrob_min = [s,qfe,qae];
lam = lambda(:,1); eta = lambda(:,2:end);
dqrob = zeros(size(qrob));
% primal-dual algorithm - primal
L = D*D';
Wf = eye(2)*2;
Wth = eye(size(qae,2))*2; 
for i=1:length(robot)
    pfe_vec = qfe(i,1:2)';
    hpme_vec = T_min{i}*...
        fkine_pme(robot(i),qfe(i,end),qae(i,:))';
    nabla_hpme = (T_min{i}*...
        Jacob_pme(robot(i),qfe(i,end),qae(i,:)))';
    upfm_vec = [-pfd(i,:)*T_min{i}';eye(2)*T_min{i}';-nabla_hpme]*Wf*(T_min{i}*...
        (pfe_vec-s(i)*pfd(i,:)')-hpme_vec);
    upfm(i,:) = upfm_vec';
    epfm(i,:) = (T_min{i}*(pfe_vec-s(i)*pfd(i,:)')-hpme_vec)';
    % test qrob_opt
    pfe_vec_opt = qfe_opt(i,1:2)';
    hpme_vec_opt = T_min{i}*...
        fkine_pme(robot(i),qfe_opt(i,end),qae_opt(i,:))';
    epfm_opt(i,:) = (T_min{i}*(pfe_vec_opt-s_opt(i)*pfd(i,:)')-hpme_vec_opt)';
end
uq = upfm+[L*lam,eta,(qae-qae_opt)*Wth];
% projection
for i=1:length(robot)
    % calculate constraint set
    A = Qnow.A{i};%[Qnow.Athfae{i};Qnow.Apfe{i}];
    b = Qnow.b{i};%[Qnow.bthfae{i};Qnow.bpfe{i}];
    qlim = Qnow.qlim{i};
    % calculate projectiojn
    if sum(~(epfm(i,:)>=-10^-4))
        savelog(logfile,['robot ' num2str(i) ' outside the sector']);
    else
        savelog(logfile,['robot ' num2str(i) ' inside the sector']);
    end
    qrob_uq(i,:)= convproj(qrob_min(i,:)-uq(i,:),A,b,qlim);
%     isinside(qrob_uq(i,:),A,b,qlim)
    if ~isinside(qrob_min(i,:),A,b,qlim)%sum(A*qrob_min(i,:)'-b>0)||sum(qrob_min(i,:)-qlim(1,:)<0)||sum(qrob_min(i,:)-qlim(2,:)>0)
        % if not inside the constraint set
        savelog(logfile,['robot ' num2str(i) ' outside the constraint set'])
        qrob_Qnow(i,:) = convproj(qrob_min(i,:),A,b,qlim);
        qrob_dist(i,:) = normalize(qrob_min(i,:)-qrob_Qnow(i,:),'norm');  
    else
        savelog(logfile,['robot ' num2str(i) ' inside the constraint set'])
        qrob_dist(i,:) = zeros(size(qrob_min(i,:)));
    end
end
dqrob_min = qrob_uq-qrob_min-kappa*qrob_dist;
% dqrob split
dqrob = dqrob_min;
ds = dqrob(:,1); dqfe = dqrob(:,2:4); 
% primal-dual algorithm - dual
dlam = L*(s+ds);
deta = (qfe-qfc_opt)+dqfe-beta*D*10*tanh(D'*eta);
% deta = (qfe-qfc_opt)+dqfe-beta*D*sign(D'*eta);
% dlambda
dlambda = [dlam,deta];
end