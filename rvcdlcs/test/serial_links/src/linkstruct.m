function link_struct = linkstruct(mass_scalar, length_scalar, link_type, q_vec, w_vec, refconfig_SE3)
% generate link data
% choose link type (R or P)
if link_type == 'R'
    xi = [-skew(w_vec)*q_vec;w_vec];
    vertices = [-length_scalar/2,0,0;length_scalar/2,0,0];
else
    xi = [w_vec;zeros(3,1)];
    vertices = [0,0,-length_scalar/2;0,0,length_scalar/2];
end
% make a matrix
link_struct.length = length_scalar;
link_struct.radius = length_scalar/10;
link_struct.twist = xi;
link_struct.vertmat = vertices';
% link_struct.xdata = vertices(:,1)';
% link_struct.ydata = vertices(:,2)';
% link_struct.zdata = vertices(:,3)';
% dynamics
M = eye(3)*mass_scalar;
r = link_struct.radius; l = link_struct.length;
I = diag([r^2/2,l^2/12+r^2/4,l^2/12+r^2/4]*mass_scalar);
link_struct.inertia = blkdiag(M,I);
link_struct.refconfig = refconfig_SE3;
link_struct.curconfig = refconfig_SE3;
% % choose reference config frame type (O or X)
% c = l/2;
% switch ref_type
%     case 'O'
%         % frame at the center of the link
%         g_lc = SE3(eye(4));
%     case '+X'
%         % inertia tensor at the end of the link
%         g_lc = SE3([c-l,0,0]);
%     case '-X'
%         % inertia tensor at the start of the link
%         g_lc = SE3([c,0,0]);
%     otherwise
%         error('Error: reference configure type should be "O", "+X", or "-X".')
% end
% link_struct.refconfig = refconfig_SE3*g_lc;
end