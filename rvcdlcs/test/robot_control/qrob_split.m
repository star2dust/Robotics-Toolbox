function varargout = qrob_split(robot,qrob,qf)


% qrob split
s = qrob(:,1); qae = qrob(:,5:end);
pfe = qrob(:,2:3); phife = qrob(:,4);
qfe = [pfe,phife];

if nargin<3
    varargout = {s,qfe,qae};
else
    % end-effector height
    robot_num = length(robot);
    for i=1:robot_num
        ze(i,:) = robot(i).altitude+robot(i).height;
        qa1_min(i,:) = robot(i).qlim(1,1);
    end
    zrpe = [ze,zeros(robot_num,2)];
    qe = q(SE2(qf)*SE2(qfe));
    [~,qe(:,end)] = circinterval(qa1_min+qf(:,end),qe(:,end));
    qa = [qe(:,end)-sum(qae,2),qae];
    % end-effector pose in SE3
    qe_rpy = [qe(:,1:2),zrpe,qe(:,end)];
    qb = robot.bkine(qa,qe_rpy);
    qc = sum(qe)/robot_num;
    varargout = {s,qfe,qae,qe,qa,qb,qc};
end
end