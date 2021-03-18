function joint_ref = suitangle(joint,joint_ref)

for j=1:length(joint(:))
    while joint(j)-joint_ref(j)>pi
        joint_ref(j)=joint_ref(j)+2*pi;
    end
    while joint(j)-joint_ref(j)<-pi
        joint_ref(j)=joint_ref(j)-2*pi;
    end
end
end