function Qmax = Q_constructor(robot,pfe_opt,s_opt,T_min,pfd,rfe,Aqa,bqa,s_lim)

import PlanarRevolute.*
robot_num = length(robot);
% Qmax - limitation (s,qfe,qae)
% rho = normby(diff(pfe_opt(convhull(pfe_opt))),1);
% Qmax.rho_lim = (rho+[-rfe,rfe])';
for i=1:robot_num
    qa_lim = robot(i).qlim;
    pfe_lim = pfe_opt(i,:)+[-rfe,rfe]';   
    thfe_lim = qa_lim(:,1);
    qfe_lim{i} = [pfe_lim,thfe_lim];
    qae_lim{i} = qa_lim(:,2:end);
end
% Qmax - pfe circle constraint
for i=1:robot_num
    Vfe{i} = circle(pfe_opt(i,:),rfe,'n',6)';
    Vfe{i} = Vfe{i}(convhull_(Vfe{i}),:);
    [Apfe{i}, bpfe{i}] = polycons(Vfe{i});
    nApfe = size(Apfe{i},1);
    Apfe{i} = [zeros(nApfe,1),Apfe{i},zeros(nApfe,3)];
end
% Qmax - joint 1 limitation
% (also to make sure T_min*pme concave)
for i=1:robot_num
    link_num = length(robot(i).link);
    Athfae{i} = [-1, ones(1,link_num-1);1, -ones(1,link_num-1)]; 
    bthfae{i} = [-robot(i).qlim(1,1);robot(i).qlim(2,1)];
    nAthfae = size(Athfae{i},1);
    Athfae{i} = [zeros(nAthfae,3),Athfae{i}];
end
% calculate a desired qa
for i=1:robot_num
    robot_link = robot(i).link;
    qa_lim = robot(i).qlim;
    qa1_min(i,:) = robot(i).qlim(1,1);
    negmup2 = @(qa) -getMu(robot_link,qa);
    qa_opt(i,:) = fmincon(negmup2,qa_lim(2,:)',Aqa{i},bqa{i},[],[],qa_lim(1,:)',...
        qa_lim(2,:)',@(qa) fkcon(robot_link,qa,pfe_opt(i,:)-s_opt*pfd(i,:)))';
end
% Qmax - pfm in sector
% (err_pfm means T_min*(pfe-pme-s*pfd))
% conditions to make err_pfm^2 convex:
% 1: T_min*pme is concave (o)
% 2: T_min*(pfe-pme-s*pfd)>=0 (x)
thfe_opt = sum(qa_opt,2);
qae_opt = qa_opt(:,2:end);
pfe_opt_eps = pfe_opt;
qrob_min_opt = [kron(ones(robot_num,1),s_opt),pfe_opt_eps,thfe_opt,qae_opt];
qth_opt = [thfe_opt,qae_opt];
% let pfe_opt_eps = pfe_opt-rfe/10*normalize(pfe_opt','norm')' to 
% make sure normby(err_pfm(robot,T_min,pfd,qrob_min_opt),1) strictly positive
if sum(normby(err_pfm(robot,T_min,pfd,qrob_min_opt),1)<=0)
    disp('err_pfm should be non-negative component wise');
end
% -T_min*pme(qth)>=-T_min*pme(qth_opt)-T_min*Jacob_pme*(qth-qth_opt) because it is convex
% T_min*(pfe-pme-s*pfd)>=T_min*(pfe-s*pfd)-T_min*pme(qth_opt)-T_min*Jacob_pme_opt*(qth-qth_opt)
% = nabla_err_pfm_opt*q_rob-T_min*pme(qth_opt)+T_min*Jacob_pme*qth_opt>=0
nabla_err_pfm_opt = nabla_err_pfm(robot,T_min,pfd,qrob_min_opt);
pme_opt = fkine_pme(robot,thfe_opt,qae_opt);
Jme_opt = Jacob_pme(robot,thfe_opt,qae_opt);
for i=1:robot_num
    Apfm{i} = -nabla_err_pfm_opt{i}';
    bpfm{i} = -T_min{i}*pme_opt(i,:)'+T_min{i}*Jme_opt{i}*[thfe_opt(i,:),qae_opt(i,:)]';
%     bpfm{i} = Apfm{i}*qrob_min_opt(i,:)';
end
% save Qmax
for i=1:robot_num
    % constaints
    Qmax.A{i} = [Apfm{i};Athfae{i};Apfe{i}];
    Qmax.b{i} = [bpfm{i};bthfae{i};bpfe{i}];
    Qmax.qlim{i} = [s_lim,qfe_lim{i},qae_lim{i}];  
end
Qmax.Apfm = Apfm; Qmax.bpfm = bpfm; 
Qmax.Athfae = Athfae; Qmax.bthfae = bthfae; 
Qmax.Apfe = Apfe; Qmax.bpfe = bpfe; Qmax.Vfe = Vfe;
Qmax.qrob_opt = qrob_min_opt;
Qmax.qa_opt = qa_opt;
Qmax.qfb_opt = [pfe_opt-getFkine(robot.lk,qa_opt),zeros(size(thfe_opt))];
Qmax.s_lim = s_lim;
end