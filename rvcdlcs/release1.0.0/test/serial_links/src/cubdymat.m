function [M,C,N]=cubdymat(rob_struct,th_vec,dth_vec,opt_cell)
% calculate dynamics matrices (M,C,N)
if nargin<4
    opt_cell = {9.8,0};
end
% parameters
Ml = {rob_struct.link.inertia};
xi = {rob_struct.link.twist};
th = th_vec;
dth = dth_vec;
g_sl0 = [rob_struct.link.refconfig];
n = length(th);
% options (to be detemined)
g = opt_cell{1};
b = opt_cell{2};
%% robot dynamics
% manipulator inertia matrix
A = cell(n,n);
% Mr = zeros(n);
for i=1:n  
    for j=1:n
        if i>=j
            % kinematics from j-th frame to i-th frame
            g_ji = gSE3(xi,th,j,i);
            % adjoint transform from j-th frame to i-th frame
            A{i,j} = Ad(inv(g_ji));
        else
            A{i,j} = zeros(6);
        end
        xi_apo{j}=Ad(SE3(expm(-hatwedge(xi{j}).*th(j))))*xi{j};
        % J_sl is always body Jacobian 
        J_sl{i}(:,j) = Ad(inv(g_sl0(i)))*A{i,j}*xi_apo{j};
    end  
    % calculate M by (4.19) (p168) and see Example 4.3 (p173) (real pages in book)
    % Mr = Mr+J_sl{i}'*Ml{i}*J_sl{i};
end
% transformed inertial matrix
Ml_apo = cell(1,n);
for i=1:n
    Ml_apo{i} = Ad(inv(g_sl0(i)))'*Ml{i}*Ad(inv(g_sl0(i)));
end
% alternative way to calculate manipulator inertia matrix
M = zeros(n);
for i=1:n
    for j=1:n
        for l=max([i,j]):n
            % calculate M by M_ij in (4.29) see Proposition 4.3 (p176)
            M(i,j) = M(i,j)+xi_apo{i}'*A{l,i}'*Ml_apo{l}*A{l,j}*xi_apo{j};         
        end
    end
end
% Christoffel symbols calculation
dM_dth = zeros(n,n,n);
Gamma = zeros(n,n,n);
for i=1:n
    for j=1:n
        for k=1:n
            for l=max([i,j]):n
                if k>=min([i,j])+1&&k<=l
                    % first calculate dM_ij/dth_k by (4.30)
                    dM_dth(i,j,k) = dM_dth(i,j,k)+bracket(A{k,i}*xi_apo{i},xi{k})'*A{l,k}'*Ml_apo{l}*A{l,j}*xi_apo{j}+xi_apo{i}'*A{l,i}'*Ml_apo{l}*A{l,k}*bracket(A{k,j}*xi_apo{j},xi{k});
                end
            end
        end
    end
end
% Coriolis matrix
C = zeros(n);
for i=1:n
    for j=1:n
        for k=1:n
            % then calculate Gamma_ijk defined in (4.23)
            Gamma(i,j,k) = (dM_dth(i,j,k)+dM_dth(i,k,j)-dM_dth(k,j,i))/2;
            % calculate Coriolis matrix
            C(i,j) = C(i,j) + Gamma(i,j,k).*dth(k);
        end
    end
end
% gravity and non-conservative force matrix
dV_dth = zeros(n,1);
for i=1:n
    g_0i = gSE3(xi,th,0,i);
    g_sl(i) = g_0i*g_sl0(i);
    com = g_sl(i)*[0,0,0]';
    ez = [0,0,1]';
    h(i) = ez'*com;
    for k=1:n
        if k<=i
            g_ki = gSE3(xi,th,k-1,i);
            dh{i}(:,k) = e2h(ez)'*g_0i.T*g_ki.inv.T*hatwedge(xi{k})*g_ki.T*g_sl0(i).T*e2h(com);
        else
            dh{i}(:,k) = 0;
        end
    end
end
N = zeros(n,1);
for k=1:n
    for i=1:n
        dV_dth(k,:) = dV_dth(k,:)+Ml{i}(1,1)*g*dh{i}(:,k);
    end
    N(k,:) = dV_dth(k,:)+b*dth(k);
end
end