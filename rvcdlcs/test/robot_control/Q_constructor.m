function Qmax = Q_constructor(robot,pfe_opt,s_opt,T_min,pfd,rfe,Aqa,bqa,s_lim)

robot_num = length(robot);
% Qmax - limitation (s,qfe,qae)
rho = normby(diff(pfe_opt(convhull(pfe_opt))),1);
Qmax.rho_lim = (rho+[-rfe,rfe])';
for i=1:robot_num
    qa_lim = robot(i).qlim;
    pfe_lim = pfe_opt(i,:)+[-rfe,rfe]';   
    thfe_lim = qa_lim(:,1);
    qfe_lim{i} = [pfe_lim,thfe_lim];
    qae_lim{i} = qa_lim(:,2:end);
end
% Qmax - pfe circle constraint
for i=1:robot_num
    Xfe = circle(pfe_opt(i,:),rfe)';
    [Apfe{i}, bpfe{i}] = polycons(Xfe);
end
% Qmax - joint 1 limitation
for i=1:robot_num
    link_num = length(robot(i).link);
    Athfae{i} = [-1, ones(1,link_num-1);1, -ones(1,link_num-1)]; bthfae{i} = robot(i).qlim(:,1);
end
% calculate a desired qa
for i=1:robot_num
    robot_link = robot(i).link;
    qa_lim = robot(i).qlim;
    negmup2 = @(qa) -PlanarRevolute.getMu(robot_link,qa);
    qa_opt(i,:) = fmincon(negmup2,qa_lim(2,:)',Aqa{i},bqa{i},[],[],...
        qa_lim(1,:)',qa_lim(2,:)',@(qa) PlanarRevolute.fkcon(robot_link,qa,pfe_opt(i,:)-s_opt*pfd(i,:)))';
end
% Qmax - pfm in sector
S = rot2(pi/2);
thfe_opt = sum(qa_opt,2);
qae_opt = qa_opt(:,2:end);
Gamma = [-ones(1,link_num-1);eye(link_num-1)];
for i=1:robot_num
    robot_link = robot(i).link;
    Apfm{i} = -T_min{i}*[-pfd(i,:)',eye(2),-S*rot2(thfe_opt(i))*...
        PlanarRevolute.getFkine(robot_link,Gamma*qae_opt(i,:)')',-rot2(thfe_opt(i))*...
        PlanarRevolute.getJacob(robot_link,Gamma*qae_opt(i,:)')*Gamma];
    bpfm{i} = Apfm{i}*[s_opt;pfe_opt(i,:)';thfe_opt(i,:);qae_opt(i,:)'];
end
for i=1:robot_num
    nApfe = size(Apfe{i},1); nAthfae = size(Athfae{i},1);
    Qmax.A{i} = [Apfm{i};zeros(nAthfae,3),Athfae{i};...
        zeros(nApfe,1),Apfe{i},zeros(nApfe,3)];
    Qmax.b{i} = [bpfm{i};bthfae{i};bpfe{i}];
    Qmax.qlim{i} = [s_lim,qfe_lim{i},qae_lim{i}];
end
end