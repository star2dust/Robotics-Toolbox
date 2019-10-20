function [G_mat, B_c_mat] = graspmap(g_oc_SE3, grasp_type) 
% calculate grasp map G
num = length(g_oc_SE3); % g_oc_SE3 is in palm frame
% detemine wrech basis
ex = [1,0,0,0,0,0]';
ey = [0,1,0,0,0,0]';
ez = [0,0,1,0,0,0]';
ox = [0,0,0,1,0,0]';
oy = [0,0,0,0,1,0]';
oz = [0,0,0,0,0,1]';
switch grasp_type % z axis as the direction of the press force
    case 'fpc' % frictionless point contact
        % f1>=0;
        B_c_mat = ez;
    case 'pcwf' % point contact with friction
        % sqrt(f1^2+f2^2)<=mu*f3; f3>=0;
        B_c_mat = [ex,ey,ez]; 
    case 'sf' % soft-finger
        % sqrt(f1^2+f2^2)<=mu*f3; f3>=0; abs(f4)<=gamma*f3
        B_c_mat = [ex,ey,ez,oz]; 
    otherwise
        % case 'firmly grasp'
        B_c_mat = [ex,ey,ez,ox,oy,oz];        
end
G_mat = [];
for i=1:num
    G_mat = [G_mat, g_oc_SE3(i).inv.Ad'*B_c_mat];
end