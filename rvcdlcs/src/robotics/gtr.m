function g_ij = gtr(xi_cell,th_vec,i,j)
% Rigid transformation g_ij from (i+1)-th frame to j-th frame if j>i otherwise eye(4)
l = min([i,j]);
u = max([i,j]);
g_lu = eye(4);
for k=l+1:u
    % if i>j then
    g_lu = g_lu*expm(wedge(xi_cell{k}).*th_vec(k));
end
if i>=j
    g_ij=invg(g_lu);
else
    g_ij=g_lu;
end
end

% if i>=j
%     % kinematics from j-th frame to i-th frame
%     g_ji = eye(4);
%     for k=j+1:i
%         % if i>j then
%         g_ji = g_ji*expm(wedge(xi_cell{k}).*th_vec(k));
%     end
%     g_ij=invg(g_ji);
% else
%     % kinematics from j-th frame to i-th frame
%     g_ij = eye(4);
%     for k=i+1:j
%         % if i>j then
%         g_ij = g_ij*expm(wedge(xi_cell{k}).*th_vec(k));
%     end
% end