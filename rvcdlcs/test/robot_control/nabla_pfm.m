function nablapfm = nabla_pfm(robot,T,pfd,thfe,qae)
link_num = size(qae,2)+1;
Gamma = [-ones(1,link_num-1);eye(link_num-1)];
S = rot2(pi/2);
for i=1:size(pfd,1)
    robot_link = robot(i).link;
    nablapfm{i} = (T{i}*[-pfd(i,:)',eye(2),-S*rot2(thfe(i))*...
        PlanarRevolute.getFkine(robot_link,Gamma*qae(i,:)')',-rot2(thfe(i))*...
        PlanarRevolute.getJacob(robot_link,Gamma*qae(i,:)')*Gamma])';
end
end