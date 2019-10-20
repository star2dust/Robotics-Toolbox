function cub_struct = cubstruct(mass_scalar, width_vec, config_SE3)
x = width_vec(1); y = width_vec(2); z = width_vec(3);
% generate cuboid data
templateVertices = [0,0,0;0,1,0;1,1,0;1,0,0;0,0,1;0,1,1;1,1,1;1,0,1];
templateFaces = [1,2,3,4;5,6,7,8;1,2,6,5;3,4,8,7;1,4,8,5;2,3,7,6];
% ^ y axis
% | 6 % % 7 -> top
% | % 2 3 % -> bottom
% | % 1 4 % -> bottom
% | 5 % % 8 -> top
% -------> x axis
cub_struct.faces = templateFaces;
cub_struct.vertices = [templateVertices(:,1)*x-x/2,templateVertices(:,2)*y-y/2,templateVertices(:,3)*z-z/2];
% configure
cub_struct.width = [x,y,z];
cub_struct.config = config_SE3;
% switch config_type
%     case 'O'
%         g_lc = SE3(eye(4));
%     case '+X'
%         g_lc = SE3(transl(x/2-x,0,0));
%     case '-X'
%         g_lc = SE3(transl(x/2,0,0));
%     case '+Y'
%         g_lc = SE3(transl(0,y/2-y,0));
%     case '-Y'
%         g_lc = SE3(transl(0,y/2,0));
%     case '+Z'
%         g_lc = SE3(transl(0,0,z/2-z));
%     case '-Z'
%         g_lc = SE3(transl(0,0,z/2));
%     otherwise
%         error('Error: Configure type should be "O", "+X", "-X", "+Y", "-Y", "+Z", or "-Z".')
% end
% cub_struct.config = config_SE3*g_lc;
% dynamics
M = eye(3)*mass_scalar;
I = diag([y^2+z^2,x^2+z^2,x^2+y^2]*mass_scalar/12);
cub_struct.inertia = blkdiag(M,I);
end