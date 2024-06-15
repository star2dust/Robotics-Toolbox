close all
clear

% initial
p0 = [0,3;-1,2;
    1,2;-2,1;
    2,1;-3,0;
    3,0;-2,-1;
    2,-1;-1,-2;
    1,-2;0,-3;];
p1 = [-1,3;1,3;
    -1,2;1,2;
    -1,1;1,1;
    -1,-1;1,-1;
    -1,-2;1,-2;
    -1,-3;1,-3;
    -2,-3;2,-3;];
figure
plot(p0(:,1),p0(:,2),'bo',p1(:,1),p1(:,2),'r*');hold on
% cost matrix
for i=1:size(p1,1)
    A(i,:) = sum((p0-p1(i,:)).*(p0-p1(i,:)),2)'.^0.5;
end
% hungarian
[assign,cost] = munkres(A);
% assign
for i=1:size(p1,1)
    if assign(i)
        plot([p0(assign(i),1),p1(i,1)],[p0(assign(i),2),p1(i,2)]);
    end
end