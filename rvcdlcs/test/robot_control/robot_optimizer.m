function [dqrob,dlambda] = robot_optimizer(robot,Qnow,D,T,pfd,qrob,lambda)

import PlanarRevolute.*
% qrob lambda split
s = qrob(:,1); pfe = qrob(:,2:3); 
thfe = qrob(:,7); qae = qrob(:,8:end);
qrob_min = [s,pfe,thfe,qae];
qae_opt = Qnow.qa_opt(:,2:end);
lam = lambda(:,1); eta = lambda(:,2:end);
link_num = size(qae,2)+1;
dqrob = zeros(size(qrob));
% primal-dual algorithm - primal
wp = 1; wth = 1; alpha = 1; beta = 2; L = D*D';
Gamma = [-ones(1,link_num-1);eye(link_num-1)];
nablapfm = nabla_pfm(robot,T,pfd,thfe,qae);
for i=1:length(robot)
    upfm(i,:) = wp*(pfe(i,:)-getFkine(robot(i).link,qae(i,:)*Gamma')...
        *rot2(thfe(i))'-s(i)*pfd(i,:))*nablapfm{i}';
end
uq = upfm+[L*lam,eta,zeros(size(lam)),wth*(qae-qae_opt)];
% projection
for i=1:length(robot)
    qrob_uq(i,:)= convproj(qrob_min(i,:)-uq(i,:),Qnow.A{i},Qnow.b{i},Qnow.qlim{i});
    qrob_Qnow(i,:) = convproj(qrob_min(i,:),Qnow.A{i},Qnow.b{i},Qnow.qlim{i});
%     qrob_uq(i,:) = lsqlin(eye(size(qrob_min,2)),qrob_min(i,:)'-uq(i,:)',...
%         Qnow.A{i},Qnow.b{i},[],[],Qnow.qlim{i}(1,:)',Qnow.qlim{i}(2,:)')';
%     qrob_Qnow(i,:) = lsqlin(eye(size(qrob_min,2)),qrob_min(i,:)',Qnow.A{i},...
%         Qnow.b{i},[],[],Qnow.qlim{i}(1,:)',Qnow.qlim{i}(2,:)')';
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