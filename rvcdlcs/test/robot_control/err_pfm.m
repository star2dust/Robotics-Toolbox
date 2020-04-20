function epfm = err_pfm(robot,T,pfd,qrob)

% is qmin
link_num = length(robot(1).link);
if size(qrob,2)==6+link_num
    qrob(:,4:6) = [];
end
% qrob split
s = qrob(:,1); pfe = qrob(:,2:3); 
thfe = qrob(:,4); qae = qrob(:,5:end);
% efm
pme = fkine_pme(robot,thfe,qae);
for i=1:length(robot)
    epfm(i,:) = (pfe(i,:)-pme(i,:)-s(i)*pfd(i,:))*T{i}';
%     epfm(i,:) = (pfe(i,:)-PlanarRevolute.getFkine(robot(i).link,qae(i,:)*Gamma')...
%         *rot2(thfe(i))'-s(i)*pfd(i,:))*T{i}';
end
end