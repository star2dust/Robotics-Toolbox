% calculate POE twist (6 x m+1) by l for mR manipulator
function xi_mat = twist(link,base)
if nargin<2
   base = zeros(1,3);
end
% choose q according to type of joints
gs0 = transl(base);
l0 = [0;link(:)];
for i=1:length(l0)
    q(:,i) = [sum(l0(1:i)),0,0]';
    q(:,i) = h2e(gs0*e2h(q(:,i)));
end
% rotation axis
w = [0,0,1]';
% joint twists 
for i=1:length(l0)-1
    xi_mat(:,i) = [-skew(w)*q(:,i);w];
end
% tool twist
g_st0 = transl(q(:,end));
xi_mat(:,i+1) = vee(logm(g_st0));
end