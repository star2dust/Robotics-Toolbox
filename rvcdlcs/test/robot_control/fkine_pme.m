function pme = fkine_pme(robot,thfe,qae)


for i=1:length(robot) 
    link_num = length(robot(i).link);
    Gamma = [-ones(1,link_num-1);eye(link_num-1)];
    pme(i,:) = PlanarRevolute.getFkine(robot(i).link,qae(i,:)*Gamma')...
        *rot2(thfe(i))';
end
end