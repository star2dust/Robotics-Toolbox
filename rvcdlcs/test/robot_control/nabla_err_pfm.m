function nablaefm = nabla_err_pfm(robot,T,pfd,qrob)

% is qmin
link_num = length(robot(1).link);
if size(qrob,2)==6+link_num
    qrob(:,4:6) = [];
end
% qrob split
thfe = qrob(:,4); qae = qrob(:,5:end);
% nabla efm
Jme = Jacob_pme(robot,thfe,qae);
for i=1:length(robot)
    robot_link = robot(i).link;
    nablaefm{i} = (T{i}*[-pfd(i,:)',eye(2),-Jme{i}])';
%     nablaefm{i} = (T{i}*[-pfd(i,:)',eye(2),-S*rot2(thfe(i))*...
%         PlanarRevolute.getFkine(robot_link,Gamma*qae(i,:)')',-rot2(thfe(i))*...
%         PlanarRevolute.getJacob(robot_link,Gamma*qae(i,:)')*Gamma])';
end
end