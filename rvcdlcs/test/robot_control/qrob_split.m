function varargout = qrob_split(robot,qrob,qf)
% is qmin
link_num = length(robot(1).link);
if size(qrob,2)==6+link_num
    qrob(:,4:6) = [];
end
% qrob split
s = qrob(:,1); qae = qrob(:,5:end);
pfe = qrob(:,2:3); thfe = qrob(:,4); 
qfe = [pfe,thfe];
if nargin==3
    qe = toqrpy(SE3.qrpy(qf).*SE3.qrpy(qfe));
    qa = [qe(:,end)-sum(qae,2),qae];
    qb = robot.bkine(qa,qe);
    robot_num = length(robot);
    qc = sum(qe)/robot_num;
    varargout = {s,pfe,thfe,qae,qa,qb,qe,qc};
else
    varargout = {s,pfe,thfe,qae};
end
end