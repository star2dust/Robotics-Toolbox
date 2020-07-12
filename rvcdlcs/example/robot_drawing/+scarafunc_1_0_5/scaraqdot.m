function qd_vec = scaraqdot(t_sca,q_vec,thr_mat,thrd_mat,thrdd_mat,t_vec,opt_cell)
import scarafunc_1_0_5.*
% the forward dynamics of scara robot in tracking control
% The function dydt = odefun(t,y), for a scalar t and a column vector y, must return a column vector dydt
nth = size(thr_mat,2);
% get current t th thd
t = t_sca;
th = q_vec(1:nth);
thd = q_vec(nth+1:2*nth);
% get current thr thrd thrdd
thr = interp1(t_vec,thr_mat,t)'; 
thrd = interp1(t_vec,thrd_mat,t)';
thrdd = interp1(t_vec,thrdd_mat,t)';
% robot dynamics matrices
m = opt_cell{1};
I = opt_cell{2};
xi = opt_cell{3};
g_sl0 = opt_cell{4};
gb = opt_cell{5};
[M,C,N] = scaradymat(m,I,xi,th,thd,g_sl0,gb);
% robot controller (e.g. PD control)
Kv = 1; Kp = 1;
tau = M*thrdd+C*thd+N+M*(-Kv*(thd-thrd)-Kp*(th-thr));
% robot forward dynamics
thdd = M^-1*(tau-C*thd-N);
% output
qd_vec = [thd(:);thdd(:)];
% if isrow(q_vec)
%     qd_vec = qd_vec';
% end
end