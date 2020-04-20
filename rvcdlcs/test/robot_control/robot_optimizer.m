function [dqrob,dlambda] = robot_optimizer(robot,Qnow,D,T,pfd,qrob,lambda)

% qrob lambda split
[s,pfe,thfe,qae] = qrob_split(robot,qrob);
qrob_min = [s,pfe,thfe,qae];
qae_opt = Qnow.qa_opt(:,2:end);
lam = lambda(:,1); eta = lambda(:,2:end);
dqrob = zeros(size(qrob));
% primal-dual algorithm - primal
wp = 1; wth = 1; alpha = 0.5; beta = 2; L = D*D';
nabla_err_pfm_now = nabla_err_pfm(robot,T,pfd,qrob_min);
err_pfm_now = err_pfm(robot,T,pfd,qrob_min);
for i=1:length(robot)
    upfm(i,:) = wp*err_pfm_now(i,:)*nabla_err_pfm_now{i}';
%     upfm(i,:) = wp*(pfe(i,:)-getFkine(robot(i).link,qae(i,:)*Gamma')...
%         *rot2(thfe(i))'-s(i)*pfd(i,:))*T{i}'*nabla_epfm_now{i}';
end
uq = upfm+[L*lam,eta,zeros(size(lam)),wth*(qae-qae_opt)];
% projection
for i=1:length(robot)
    % calculate constraint set
    A = Qnow.A{i};%[Qnow.Athfae{i};Qnow.Apfe{i}];
    b = Qnow.b{i};%[Qnow.bthfae{i};Qnow.bpfe{i}];
    qlim = Qnow.qlim{i};
    % calculate projectiojn
%     if sum(~(A*qrob_min(i,:)'-b<0))
%         disp('outside the constraint set')
%     else
%         disp('inside the constraint set')
%     end
%     if sum(normby(err_pfm(robot,T,pfd,qrob),1)<=0)
%         disp('outside the sector');
%     else
%         disp('inside the sector');
%     end
    qrob_uq(i,:)= convproj(qrob_min(i,:)-uq(i,:),A,b,qlim);
    qrob_Qnow(i,:) = convproj(qrob_min(i,:),A,b,qlim);
    if norm(qrob_min(i,:)-qrob_Qnow(i,:))<=10^-4
        qrob_dist(i,:) = zeros(size(qrob_min(i,:)));
    else
        qrob_dist(i,:) = normalize(qrob_min(i,:)-qrob_Qnow(i,:),'norm');
    end
end
dqrob_min = qrob_uq-qrob_min-alpha*qrob_dist;
% dqrob split
ds = dqrob(:,1); dpfe = dqrob(:,2:3); 
dqrob(:,[1:3,7:end]) = dqrob_min;
% primal-dual algorithm - dual
dlam = L*(s+ds);
deta = pfe+dpfe-beta*D*sign(D'*eta);
% dlambda
dlambda = [dlam,deta];
end