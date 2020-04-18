close all
clear


r = 1;
x = [cos(pi/3), cos(pi/2), cos(pi), cos(-pi*2/3);
    sin(pi/3), sin(pi/2), sin(pi), sin(-pi*2/3)];
y = [cos(0), cos(pi/2), cos(pi), cos(-pi/2);
    sin(0), sin(pi/2), sin(pi), sin(-pi/2)];
% plot(y(1,:),y(2,:),'bo');

A = [0 1 1 1;
    1 0 1 0;
    1 1 0 1;
    1 0 1 0];
D = diag(sum(A,2));
L = D-A;

sumPg = zeros(8,2);
for i=1:4
    for j=1:4
        if i~=j
            gij = (y(:,j)-y(:,i))/norm(y(:,j)-y(:,i));
            Pg(2*i-1:2*i,2*j-1:2*j) = A(i,j)*(eye(2)-gij*gij');
        else
            Pg(2*i-1:2*i,2*j-1:2*j)=zeros(2,2);
        end
        sumPg(2*i-1:2*i,:) = sumPg(2*i-1:2*i,:) + Pg(2*i-1:2*i,2*j-1:2*j);
    end
end
for i=1:4
    for j=1:4
        if i==j
            B(2*i-1:2*i,2*j-1:2*j)=sumPg(2*i-1:2*i,:);
        else
            B(2*i-1:2*i,2*j-1:2*j)=-Pg(2*i-1:2*i,2*j-1:2*j);
        end
    end
end

y = x;
z = x;
u = zeros(size(x));
v = u;
dt = 0.05;
w = kron(ones(4,1),eye(2));
figure
for i=1:500
%     w = diag(1./sqrt(diag(z'*z)));dz = -5*kron(w,eye(2))*z(:)-kron(L,eye(2))*u(:);
    dx = -1*(x(:)-y(:))-B*v(:);
    dz = -3*z(:)-kron(L,eye(2))*u(:);
    du = kron(L,eye(2))*(x(:)+z(:));
    dv = B*x(:);
    z(:) = z(:) + dz*dt;
    u(:) = u(:) + du*dt;
    x(:) = x(:) + dx*dt;
    v(:) = v(:) + dv*dt;
    plot(y(1,:),y(2,:),'go');axis([-1 1 -1 1]*2);hold on
    plot(x(1,:),x(2,:),'bo');
    plot(x(1,:)+z(1,:),x(2,:)+z(2,:),'ro');hold off
    pause(dt);
end