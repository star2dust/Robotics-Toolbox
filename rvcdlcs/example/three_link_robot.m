% three-link robot kinematics and dynmaics calculation
% reference book: Richard M. Murray-A Mathematical Introduction to Robotic Manipulation-CRC Press (1994). 
close all
clear

%% parameters
% link length
syms l0 l1 l2 real
% link center of mass
syms r0 r1 r2 real
% joint angle
syms th1 th2 th3 real
% joint velocity
syms dth1 dth2 dth3 real
% link mass
syms m1 m2 m3 real
% link inertia
syms Ix1 Iy1 Iz1 Ix2 Iy2 Iz2 Ix3 Iy3 Iz3 real
%% reference config
g_st0 = transl(0,l1+l2,l0);
% choose q according to type of joints (page 193)
q1 = [0,0,0]';
q2 = [0,0,l0]';
q3 = [0,l1,l0]';
w1 = [0,0,1]';
w2 = [-1,0,0]';
w3 = w2;
% joint twists
xi1 = [-skew(w1)*q1;w1];
xi2 = [-skew(w2)*q2;w2];
xi3 = [-skew(w3)*q3;w3];
% reference config for each joint
g_sl0{1} = transl(0,0,r0);
g_sl0{2} = transl(0,r1,l0);
g_sl0{3} = transl(0,l1+r2,l0);
% make a matrix
xi = {xi1,xi2,xi3};
th = [th1,th2,th3];
dth = [dth1,dth2,dth3];
m = [m1,m2,m3];
I = {[Ix1 Iy1 Iz1],[Ix2 Iy2 Iz2],[Ix3 Iy3 Iz3]};
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
                g_ji = g_ji*expm(hatwedge(xi{k}).*th(k));
            end
            % adjoint transform from j-th frame to i-th frame
            A{i,j} = Adg(invg(g_ji));
        else
            A{i,j} = zeros(6);
        end
        xi_apo{j}=Adg(expm(-hatwedge(xi{j}).*th(j)))*xi{j};
        % J_sl is always body Jacobian 
        J_sl{i}(:,j) = Adg(invg(g_sl0{i}))*A{i,j}*xi_apo{j};
        J_sl{i}(:,j) = simplify(J_sl{i}(:,j));
    end  
    % calculate M by (4.19) (p168) and see Example 4.3 (p173)
    Mr = Mr+J_sl{i}'*Ml{i}*J_sl{i};
end
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
% Christoffel symbols for test (reference from p194)
M11 = Iz1 + (m2*r1^2*cos(th2)^2 + Iz2*cos(th2)^2 + Iy2*sin(th2)^2) + (m3*(l1*cos(th2)+r2*cos(th2+th3))^2 + Iy3*sin(th2+th3)^2 + Iz3*cos(th2+th3)^2);
Gammar112 = (Iy2-Iz2-m2*r1^2)*cos(th2)*sin(th2)+(Iy3-Iz3)*cos(th2+th3)*sin(th2+th3)-m3*(l1*cos(th2)+r2*cos(th2+th3))*(l1*sin(th2)+r2*sin(th2+th3));
Gammar113 = (Iy3-Iz3)*cos(th2+th3)*sin(th2+th3)-m3*r2*sin(th2+th3)*(l1*cos(th2)+r2*cos(th2+th3));
dM11_dth2 = Iy2*2*sin(th2)*cos(th2)+Iy3*2*sin(th2+th3)*cos(th2+th3)-Iz2*2*cos(th2)*sin(th2)-Iz3*2*cos(th2+th3)*sin(th2+th3)-m2*r1^2*2*cos(th2)*sin(th2)+m3*2*(l1*cos(th2)+r2*cos(th2+th3))*(-l1*sin(th2)-r2*sin(th2+th3));
dM11_dth3 = Iy3*2*sin(th2+th3)*cos(th2+th3)-Iz3*2*cos(th2+th3)*sin(th2+th3)-m3*2*(l1*cos(th2)+r2*cos(th2+th3))*r2*sin(th2+th3);
dM12_dth1 = 0;
dM13_dth1 = 0;
dM21_dth1 = 0;
dM31_dth1 = 0;
Gamma112 = (dM11_dth2+dM12_dth1-dM21_dth1)/2;
Gamma113 = (dM11_dth3+dM13_dth1-dM31_dth1)/2;
g_31 = expm(-hatwedge(xi{3}.*th(3)))*expm(-hatwedge(xi{2}.*th(2)));
A31 = Adg(g_31);
dA31xi1_apo = bracket(A31*xi_apo{1},xi{3}); % hatvee(-hatwedge(xi{3})*g_31*hatwedge(xi_apo{1})*invg(g_31)+g_31*hatwedge(xi_apo{1})*invg(g_31)*hatwedge(xi{3}));
dM_dth113 = dA31xi1_apo'*Ml_apo{3}*A31*xi_apo{1}+xi_apo{1}'*A31'*Ml_apo{3}*dA31xi1_apo;
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
% gravity and non-conservative force matrix
g_sl=cell(1,n);
h=cell(1,n);
g_0i=eye(4);
for i=1:n
    g_0i = g_0i*expm(hatwedge(xi{i}).*th(i));
    g_sl{i} = g_0i*g_sl0{i};
    [~,p_t] = tr2rt(g_sl{i});
    wz = [0,0,1]';
    h{i}=p_t'*wz;
end
