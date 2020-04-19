function nablaefm = nabla_epfm(robot,T,pfd,qrob)

% is qmin
link_num = length(robot(1).link);
if size(qrob,2)==7+link_num
    qrob(:,4:6) = [];
end
% qrob split
thfe = qrob(:,4); qae = qrob(:,5:end);
% nabla efm
Gamma = [-ones(1,link_num-1);eye(link_num-1)];
S = rot2(pi/2);
for i=1:length(robot)
    robot_link = robot(i).link;
    nablaefm{i} = (T{i}*[-pfd(i,:)',eye(2),-S*rot2(thfe(i))*...
        PlanarRevolute.getFkine(robot_link,Gamma*qae(i,:)')',-rot2(thfe(i))*...
        PlanarRevolute.getJacob(robot_link,Gamma*qae(i,:)')*Gamma])';
end
end