function Qmax = Q_constructor(robot,qfe,qa_opt,T_min,pfd,xi,s_lim)

import PlanarRevolute.*
robot_num = length(robot);
% Qmax - range set (s,qfe,qae)
for i=1:robot_num
    qa_lim = robot(i).qlim;
    pfe_lim = qfe(i,1:2)+[-xi,xi]';   
    qfe_lim{i} = [pfe_lim,qa_lim(:,1)];
    qae_lim{i} = qa_lim(:,2:end);
end
% Qmax - pfe circle constraint => polygon
poly_num = 6;
poly = nsidedpoly(poly_num);
for i=1:robot_num
    Vfe{i} = qfe(i,1:2)+poly.Vertices*xi/cos(pi/poly_num);
    Vfe{i} = Vfe{i}(convhull_(Vfe{i}),:);
    [Apfe{i}, bpfe{i}] = polycons(Vfe{i});
    nApfe = size(Apfe{i},1);
    Apfe{i} = [zeros(nApfe,1),Apfe{i},zeros(nApfe,3)];
end
% Qmax - joint 1 limitation
% (also to make sure T_min*pme concave)
for i=1:robot_num
    link_num = length(robot(i).link)-1;
    link_tril = tril(ones(link_num));
    Aqfae{i} = [ones(link_num,1),-link_tril(:,end:-1:1);
        -ones(link_num,1),link_tril(:,end:-1:1)];
    bqfae{i} = [kron(ones(link_num,1),robot(i).qlim(2,1));
        kron(ones(link_num,1),-robot(i).qlim(1,1));];
    nAthfae = size(Aqfae{i},1);
    Aqfae{i} = [zeros(nAthfae,3),Aqfae{i}];
end
% Qmax - pfm in sector
% 1: T_min*pme is concave 
% 2: T_min*(pfe-s*pfd)-T_min*pme>=0 
phife_opt = sum(qa_opt,2);
qae_opt = qa_opt(:,2:end);
% pfe_opt_eps = pfe_opt;
% qrob_min_opt = [kron(ones(robot_num,1),s_opt),pfe_opt_eps,phife_opt,qae_opt];
% qth_opt = [phife_opt,qae_opt];

% let pfe_opt_eps = pfe_opt-rfe/10*normalize(pfe_opt','norm')' to 
% make sure normby(err_pfm(robot,T_min,pfd,qrob_min_opt),1) strictly positive
% if sum(normby(err_pfm(robot,T_min,pfd,qrob_min_opt),1)<=0)
%     disp('err_pfm should be non-negative component wise');
% end
% -T_min*pme(qth)>=-T_min*pme(qth_opt)-T_min*Jacob_pme*(qth-qth_opt) because it is convex
% T_min*(pfe-pme-s*pfd)>=T_min*(pfe-s*pfd)-T_min*pme(qth_opt)-T_min*Jacob_pme_opt*(qth-qth_opt)
% = nabla_err_pfm_opt*q_rob-T_min*pme(qth_opt)+T_min*Jacob_pme*qth_opt>=0
% nabla_err_pfm_opt = nabla_err_pfm(robot,T_min,pfd,qrob_min_opt);
% pme_opt = fkine_pme(robot,phife_opt,qae_opt);
% Jme_opt = Jacob_pme(robot,phife_opt,qae_opt);
for i=1:robot_num
    hpme_opt_vec = T_min{i}*...
        fkine_pme(robot(i),phife_opt(i,:),qae_opt(i,:))';
    nabla_hpme_opt = (T_min{i}*...
        Jacob_pme(robot(i),phife_opt(i,:),qae_opt(i,:)))';
    Apfm{i} = -[T_min{i}*[-pfd(i,:)', eye(2)], -nabla_hpme_opt'];
    bpfm{i} = -hpme_opt_vec+...
        nabla_hpme_opt'*[phife_opt(i,:)';qae_opt(i,:)'];
%     -T_min{i}*pme_opt(i,:)'+T_min{i}*Jme_opt{i}*[phife_opt(i,:),qae_opt(i,:)]';
%     bpfm{i} = Apfm{i}*qrob_min_opt(i,:)';
end
% save Qmax
for i=1:robot_num
    % constaints
    Qmax.A{i} = [Apfm{i};Aqfae{i};Apfe{i}];
    Qmax.b{i} = [bpfm{i};bqfae{i};bpfe{i}];
    Qmax.qlim{i} = [s_lim,qfe_lim{i},qae_lim{i}];  
end
Qmax.Apfm = Apfm; Qmax.bpfm = bpfm; 
Qmax.Aqfae = Aqfae; Qmax.bqfae = bqfae; 
Qmax.Apfe = Apfe; Qmax.bpfe = bpfe; 
Qmax.Vfe = Vfe;
Qmax.s_lim = s_lim;
Qmax.qfe_lim = qfe_lim;
Qmax.qae_lim = qae_lim;
Qmax.phife_opt = phife_opt;
Qmax.qae_opt = qae_opt;
% Qmax.qrob_opt = qrob_min_opt;
% Qmax.qa_opt = qa_opt;
% Qmax.qfb_opt = [pfe_opt-getFkine(robot.lk,qa_opt),zeros(size(phife_opt))];
end