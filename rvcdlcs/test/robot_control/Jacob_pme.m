function Jme = Jacob_pme(robot,thfe,qae)

S = rot2(pi/2);
for i=1:length(robot)
    link_num = length(robot(i).link);
    robot_link = robot(i).link;
    Gamma = [-ones(1,link_num-1);eye(link_num-1)];
    Jme(:,:,i) = [S*rot2(thfe(i))*PlanarRevolute.getFkine(robot_link,Gamma*qae(i,:)')',...
        rot2(thfe(i))*PlanarRevolute.getJacob(robot_link,Gamma*qae(i,:)')*Gamma];
end
end