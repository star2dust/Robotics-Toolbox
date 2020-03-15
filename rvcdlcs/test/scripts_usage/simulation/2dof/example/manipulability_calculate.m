close all
clear
clc

% define m dof mR manipulator
m = 3;
% dx: dxdthi, dy: dydthi
dx = sym('dx',[m,1],'real');
dy = sym('dy',[m,1],'real');
% calculate
ls = sym('ls',[m,1],'real');
lc = sym('lc',[m,1],'real');
th = sym('th',[m,1],'real');
l = sym('l',[m,1],'real');
% l = ones(m,1);
% lsin and lcos
for i=1:m
    for j=1:m
        ls(i) = l(i)*sin(sum(th(1:i)));
        lc(i) = l(i)*cos(sum(th(1:i)));
    end
end
% dxdth and dydth
for i=1:m
    dx(i) = 0;
    dy(i) = 0;
    for j=i:m
        dx(i) = dx(i)-ls(j);
        dy(i) = dy(i)+lc(j);
    end
end
% Jacobian
J = [dx,dy]';
% simplify the expression det(JJ')
JJT = zeros(2,2);
trJJT = 0;
for i=1:m
    JJT = JJT+[dx(i)^2,dx(i)*dy(i);dx(i)*dy(i),dy(i)^2];
    trJJT = trJJT+dx(i)^2+dy(i)^2;
end
% simplify(det(JJT))
% simplify(trJJT)
% mu^2
mupower2 = 0;
for i=1:m
    for j=1:m
        mupower2 = mupower2 + dx(i)^2*dy(j)^2-dx(i)*dy(i)*dx(j)*dy(j);
%         simplify(dx(i)^2*dy(j)^2-dx(i)*dy(i)*dx(j)*dy(j))
    end
end
simplify(mupower2)
% forward kinematics
f = [dy(1);-dx(1)];
% Hessian
H(:,:,1) = [-dy,dx]';
for i=2:m
    H(:,:,i) = H(:,:,i-1);
    for j=1:i-1
        H(:,j,i) = H(:,i,i);
    end
end
% row vec of Hessian
for i=1:m
    Htrans = H(:,:,i)';
    rvecH(i,:)= Htrans(:)';
end
rvecH1 = rvecH;
% second method to calcuate row vec of Hessian
Hdy = kron(ones(1,m),dy);
Hdx = kron(ones(1,m),dx);
for i=1:m-1
    for j=i:m
        Hdy(i,j) = dy(j);
        Hdx(i,j) = dx(j);
    end
end
rvecH2 = [-Hdy,Hdx];
% nabla mu
JJTinv = JJT^-1;
nablamu = sqrt(det(JJT))*rvecH*kron(eye(2),J')*JJTinv(:);
simplify(nablamu)

