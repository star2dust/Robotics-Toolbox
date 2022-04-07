function [z,exit] = poly2nodez(polytope,rloc,config,envir,robotnum,dispbaseref,dispendref)
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
% width = max(abs(envir.ub-envir.lb))/4;
% sliwin.lb = rloc-width;
% sliwin.ub = rloc+width;
% lb = [max(envir.lb,sliwin.lb),config.lb];
% ub = [min(envir.ub,sliwin.ub),config.ub];
lb = [envir.lb,config.lb];
ub = [envir.ub,config.ub];
% calculate z (row vec)
W = diag([1,1,10]);
[z,~,exit] = fmincon(@(z) (z-zd)*W*(z-zd)',zd,[],[],[],[],lb,ub,@(z) safecon(z,Ap,bp,robotnum,dispbaseref,dispendref));
end

function [c,ceq] = safecon(z,Ap,bp,robotnum,dispbaseref,dispendref)

% get rloc and config (siz)
rloc = z(1:2);
siz = z(3);
% you may change ang according to rloc here
% get displacement base
rob_vts = zeros(robotnum,2);
for i=1:robotnum
    rob_vts(i,:) = rloc+(dispendref(i,:)+(dispbaseref(i,:)-dispendref(i,:))*siz);
end
% final constraints
c = max(vec(Ap*rob_vts'-bp));
ceq = 0;
end