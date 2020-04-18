function xi = MmRtwist(l)
% choose q according to type of joints
l0 = [0;l(1:end-1)];
for i=1:length(l0)
q(:,i) = [sum(l0(1:i)),0,0]';
end
w = [0,0,1]';
% platform
v1 = [1,0,0]';
v2 = [0,1,0]';
% joint twists (plt=>1:3,slk=>4:6)
xi{1} = [v1;zeros(3,1)];
xi{2} = [v2;zeros(3,1)];
xi{3} = [-skew(w)*q(:,1);w];
for i=1:length(l0)
xi{i+3} = [-skew(w)*q(:,i);w];
end
end