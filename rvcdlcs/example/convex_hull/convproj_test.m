close all
clear


% constraint polygon
Vc = rand(5,2);
[A,b,Aeq,beq] = polycons(Vc);

Vc = Vc(convhull_(Vc),:);
plot(Vc(:,1),Vc(:,2),'g-'); hold on
axis([0 1 0 1]);


% test points
V0 = rand(5,2);
lim = [0,0;1,1];
[Vp,resn,flag] = convproj(V0,A,b,lim,Aeq,beq);
for i=1:size(V0)
    plot([V0(i,1),Vp(i,1)],[V0(i,2), Vp(i,2)],'ro-');
end