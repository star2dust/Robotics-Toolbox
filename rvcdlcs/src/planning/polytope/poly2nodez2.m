function [z,exit] = poly2nodez2(polytope,rloc,config,envir,robot_num,disp_base_max,disp_end_ref)
% Find a feasible node z in one or two polytopes

% get the A and b from polytope
Ap = []; bp = [];
for i=1:length(polytope)
    Ap = [Ap;polytope(i).A];
    bp = [bp;polytope(i).b];
end
% nodez desired
zd = [rloc,config.vec];
% constraints lb and ub
ab = anglelim(rloc(1:2),envir);
lb = [envir.lb,ab(1),config.lb];
ub = [envir.ub,ab(2),config.ub];
% calculate z (row vec)
W = diag([1,1,10,10]);
[z,~,exit] = fmincon(@(z) (z-zd)*W*(z-zd)',zd,[],[],[],[],lb,ub,@(z) safecon(z,Ap,bp,robot_num,disp_base_max,disp_end_ref));
end

function [c,ceq] = safecon(z,Ap,bp,robot_num,disp_base_max,disp_end_ref)

% get rloc and config (siz)
rloc = z(1:3);
siz = z(end);
% get displacement base
rob_vts = zeros(robot_num,2);
for i=1:robot_num
    rob_vts(i,:) = rloc(1:2)+(disp_end_ref(i,:)+(disp_base_max(i,:)-disp_end_ref(i,:))*siz)*rot2(rloc(3))';
end
% final constraints
c = max(vec(Ap*rob_vts'-bp));
ceq = 0;
end