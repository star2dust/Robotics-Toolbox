function lincon_plot(A,b,lb,ub)
% plot a linear constraint A*x<=b to test whether it is valid
r = lb:(ub-lb)/20:ub;
x = kron(r,ones(size(r)));
y = kron(ones(size(r)),r);
p = [x;y];
for i=1:size(p,2)
    if max(A*p(:,i)>b)
        p(:,i)=nan;
    end
end
ish = ishold();
if ~ishold
    hold on
end
plot(p(1,:),p(2,:),'o');
% restore hold setting
if ~ish
    hold off
end
end