% SCARA robot kinematics and dynmaics calculation
% reference book: Richard M. Murray-A Mathematical Introduction to Robotic Manipulation-CRC Press (1994). 
close all
clear

%% parameters
% link length
syms l0 l1 l2 l3 real
% link center of mass
syms r1 r2 r3 real
% joint angle
syms th1 th2 th3 th4 real
% joint velocity
syms dth1 dth2 dth3 dth4 real
% link mass
syms m1 m2 m3 m4 real
% link inertia
syms Ix1 Iy1 Iz1 Ix2 Iy2 Iz2 Ix3 Iy3 Iz3 Ix4 Iy4 Iz4 real
%% reference config
g_st0 = transl(0,l1+l2,l0);
% choose q according to type of joints
q1 = [0,0,0]';
q2 = [0,l1,0]';
q3 = [0,l1+l2,0]';
w1 = [0,0,1]';
w2 = w1; w3 = w1; v4 = w1;
% joint twists
xi1 = [-skew(w1)*q1;w1];
xi2 = [-skew(w2)*q2;w2];
xi3 = [-skew(w3)*q3;w3];
xi4 = [v4;zeros(3,1)];
% reference config for each joint
g_sl0{1} = transl(0,r1,l0+r3);
g_sl0{2} = transl(0,l1+r2,l0+r3);
g_sl0{3} = transl(0,l1+l2,l0+r3);
g_sl0{4} = transl(0,l1+l2,l0);
% make a matrix
xi = {xi1,xi2,xi3,xi4};
th = [th1,th2,th3,th4];
dth = [dth1,dth2,dth3,dth4];
m = [m1,m2,m3,m4];
I = {[Ix1 Iy1 Iz1],[Ix2 Iy2 Iz2],[Ix3 Iy3 Iz3],[Ix4,Iy4,Iz4]};
n = length(th);
%% robot dynamics
for i=1:n
    % inertia matrix for i-th link
    Ml{i} = blkdiag(m(i)*eye(3),diag(I{i}));
end
% manipulator inertia matrix
A = cell(n,n);
Mr = zeros(n);
for i=1:n  
    for j=1:n
        if i>=j
            % kinematics from j-th frame to i-th frame
            g_ji = eye(4);
            for k=j+1:i
                % if i>j then
                g_ji = g_ji*expm(wedge(xi{k}).*th(k));
            end
            % adjoint transform from j-th frame to i-th frame
            A{i,j} = Adg(invg(g_ji));
        else
            A{i,j} = zeros(6);
        end
        xi_apo{j}=Adg(expm(-wedge(xi{j}).*th(j)))*xi{j};
        % J_sl is always body Jacobian 
        J_sl{i}(:,j) = Adg(invg(g_sl0{i}))*A{i,j}*xi_apo{j};
        J_sl{i}(:,j) = simplify(J_sl{i}(:,j));
    end  
    % calculate M by (4.19) (p168) and see Example 4.3 (p173)
    Mr = Mr+J_sl{i}'*Ml{i}*J_sl{i};
end
% test Mr here by simplify(Mr-Mr_t) (reference from p199)
a_t = Iz1+r1^2*m1+l1^2*m2+l1^2*m3+l1^2*m4;
b_t = Iz2+Iz3+Iz4+l2^2*m3+l2^2*m4+m2*r2^2;
c_t = l1*l2*m3+l1*l2*m4+l1*m2*r2;
d_t = Iz3+Iz4;
Mr_t = [a_t+b_t+2*c_t*cos(th2),b_t+c_t*cos(th2),d_t,0;
    b_t+c_t*cos(th2),b_t,d_t,0;
    d_t,d_t,d_t,0;
    0,0,0,m4];
% transformed inertial matrix
Ml_apo = cell(1,n);
for i=1:n
    Ml_apo{i} = Adg(invg(g_sl0{i}))'*Ml{i}*Adg(invg(g_sl0{i}));
end
% alternative way to calculate manipulator inertia matrix
M = sym(zeros(n));
for i=1:n
    for j=1:n
        for l=max([i,j]):n
            % calculate M by M_ij in (4.29) see Proposition 4.3 (p176)
            % check whether M is equal to M1 by simplify(M-M1)
            M(i,j) = M(i,j)+xi_apo{i}'*A{l,i}'*Ml_apo{l}*A{l,j}*xi_apo{j};         
        end
    end
end
% Christoffel symbols calculation
dM_dth = sym(zeros(n,n,n));
Gamma = sym(zeros(n,n,n));
for i=1:n
    for j=1:n
        for k=1:n
            for l=max([i,j]):n
                if k>=min([i,j])+1&&k<=l
                    % first calculate dM_ij/dth_k by (4.30)
                    % check whether dM_dth(1,1,2) is equal to dM11_dth2 by simplify(dM_dth(1,1,2)-dM11_dth2)
                    dM_dth(i,j,k) = dM_dth(i,j,k)+bracket(A{k,i}*xi_apo{i},xi{k})'*A{l,k}'*Ml_apo{l}*A{l,j}*xi_apo{j}+xi_apo{i}'*A{l,i}'*Ml_apo{l}*A{l,k}*bracket(A{k,j}*xi_apo{j},xi{k});
                end
            end
        end
    end
end
% Coriolis matrix
C = sym(zeros(n));
for i=1:n
    for j=1:n
        for k=1:n
            % then calculate Gamma_ijk defined in (4.23)
            % check whether Gamma(1,1,2) is equal to Gammar112 by simplify(Gamma(1,1,2)-Gammar112)
            Gamma(i,j,k) = (dM_dth(i,j,k)+dM_dth(i,k,j)-dM_dth(k,j,i))/2;
            % calculate Coriolis matrix
            C(i,j) = C(i,j) + Gamma(i,j,k).*dth(k);
        end
    end
end
% test C here by simplify(C-C_t) (reference from p199)
C_t = [-c_t*sin(th2)*dth2,-c_t*sin(th2)*(dth1+dth2),0,0;
    c_t*sin(th2)*dth1,0,0,0;
    0,0,0,0;
    0,0,0,0];