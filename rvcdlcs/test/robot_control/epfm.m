function efm = epfm(robot,T,pfd,qrob)

% is qmin
link_num = length(robot(1).link);
if size(qrob,2)==6+link_num
    qrob(:,4:6) = [];
end
% qrob split
s = qrob(:,1); pfe = qrob(:,2:3); 
thfe = qrob(:,4); qae = qrob(:,5:end);
% efm
Gamma = [-ones(1,link_num-1);eye(link_num-1)];
for i=1:length(robot)
    efm(i,:) = (pfe(i,:)-PlanarRevolute.getFkine(robot(i).link,qae(i,:)*Gamma')...
        *rot2(thfe(i))'-s(i)*pfd(i,:))*T{i}';
end
end