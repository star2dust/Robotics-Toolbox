function th_mat = ikine4(l_vec, tool_SE3, elbow_type)
% get inverse kinematics of scara robot from tool frame (SE3)

if nargin<3
    elbow_type = 'de';
end

l1 = l_vec(1); l2 = l_vec(2); l3 = l_vec(3); l4 = l_vec(4);
n = length(tool_SE3);
for i=1:n
    end_SE3 = tool_SE3(i)*SE3([-l4,0,0]);
    [RT, pT] = tr2rt(end_SE3);
    thT = vex(logm(RT));
    x = pT(1); y = pT(2); z = pT(3);
    % angle => inverse kinematics of three-link planar robot
    switch elbow_type
        case 'ue' % up-elbow
            ue = -1;
        case 'de' % down-elbow
            ue = 1;
    end
    c2 = (x^2+y^2-l1^2-l2^2)/(2*l1*l2);
    th2 = acos(c2)*ue;
    s2 = sqrt(1-c2^2);
    th1 = atan2(y,x)-atan2(l2*s2,l1+l2*c2)*ue;
    th4 = thT(3)-th1-th2;
    % height
    th3 = l3 - z;
    th_mat(i,:) = [th1, th2, th3, th4];
end
end