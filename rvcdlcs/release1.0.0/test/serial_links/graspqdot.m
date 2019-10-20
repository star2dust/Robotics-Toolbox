function qd_vec = graspqdot(t_scalar, q_vec, xr_mat, xrd_mat, xrdd_mat, tspan_vec, rob_struct, obj_struct)
%% the forward dynamics of scara robot in grasp control
persistent t_prev J_inv_GT_prev
% The function dydt = odefun(t,y), for a scalar t and a column vector y, must return a column vector dydt
nx = size(xr_mat,2);
nrob = length(rob_struct);
% get current t th thd
t = t_scalar;
x = q_vec(1:nx);
xd = q_vec(nx+1:2*nx);
nth = 0; idx = 2*nx;
for i=1:nrob
    idx = idx + 2*nth;
    nth = length(rob_struct(i).link);
    th(:,i) = q_vec(idx+1:idx+nth);
    thd(:,i) = q_vec(idx+nth+1:idx+2*nth);
end
% get current thr thrd thrdd
xr = interp1(tspan_vec,xr_mat,t)'; 
xrd = interp1(tspan_vec,xrd_mat,t)';
xrdd = interp1(tspan_vec,xrdd_mat,t)';
% renew current robot and object states
obj_struct.config = SE3(x(1:3))*SE3.rpy(x(4:6)');
for i=1:nrob
    % renew robot tool (in spatial frame) and g_sc
    rob_struct(i).tool = obj_struct.config*obj_struct.g_oc(i); % g_pc = g_po*g_oc
    % relative config g_sc
    rob_struct(i).g_sc = rob_struct(i).base.inv*rob_struct(i).tool; % assuming that g_fc = SE3(eye(4))
end
%% two scara fingers grasping a box => get J_h (page 261)
% Jacobian for a SCARA robot (page 139)
for i=1:nrob
   rob_struct(i).J_sf_s(:,1) = rob_struct(i).link(1).twist;
   for j=2:length(rob_struct(i).link)
       rob_struct(i).J_sf_s(:,j) = Ad(gSE3({rob_struct(i).link.twist},th(:,i),0,j-1))*rob_struct(i).link(j).twist;
   end
end
% Adjoint tranformation associated with g_sf(g_sc)
obj_struct.J_h = [];
for i=1:nrob
    obj_struct.J_h = blkdiag(obj_struct.J_h, obj_struct.B_c'*Ad(inv(rob_struct(i).g_sc))*rob_struct(i).J_sf_s);
end
%% update variables
g = [0,0,9.8]'; w = xd(4:6); R_po = obj_struct.config.R;
mo = obj_struct.inertia(1,1); Io = obj_struct.inertia(4:6,4:6);
G = obj_struct.G; J = obj_struct.J_h;
%% dynamics matrices
% object dynamics matrices
Mo = blkdiag(mo*eye(3),Io); 
Co = blkdiag(mo*skew(w),skew(w)*Io);
No = [R_po'*mo*g;zeros(3,1)];
% robot dynamics matrices
Mf = []; Cf = []; Nf = [];
for i=1:nrob
    [M,C,N]=dynmat(rob_struct(i),th(:,i),thd(:,i));
    Mf = blkdiag(Mf,M);
    Cf = blkdiag(Cf,C);
    Nf = [Nf;N];
end
% grasp system dynamics matrices
if t~=t_prev
    J_inv_GT_dot = (pinv(J)*G'-J_inv_GT_prev)/(t-t_prev);
else
    J_inv_GT_dot = zeros(size(pinv(J)*G'));
end
M = Mo+G*pinv(J')*Mf*pinv(J)*G';
C = Co+G*pinv(J')*(Cf*pinv(J)*G'+Mf*J_inv_GT_dot);
N = No+G*pinv(J')*Nf;
% robot controller (e.g. PD control)
Kv = 1; Kp = 1;
F = M*xrdd+C*xd+N+M*(-Kv*(xd-xrd)-Kp*(x-xr));
fN = [1,0,0,0,1,0,0,0]';
tau = J'*pinv(G)*F+J'*fN;
% robot forward dynamics
xdd = M^-1*(F-C*xd-N);
thdd_temp = Mf^-1*(tau-Cf*thd(:)-Nf);
% output
idx = 0;
for i=1:nrob
    thdd(:,i) = thdd_temp(idx+1:idx+length(rob_struct(i).link));
    idx = length(rob_struct(i).link);
end
thdthdd = [thd;thdd];
qd_vec = [xd(:);xdd(:);thdthdd(:)];
% reset persistent variables
t_prev = t;
J_inv_GT_prev = pinv(J)*G';
end