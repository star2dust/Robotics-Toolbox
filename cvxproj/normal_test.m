close all
clear

% normal vector
vn = [1;1;1];
vz = [0;0;1]; % before rotation
% vz to nv => rotation angle and axis
th_zn = acos(vz'*vn/(norm(vn)*norm(vz)));
w_zn = skew(vz)*vn;
w_zn = w_zn/norm(w_zn);
% Rodrigues' rotation formula
R_zn = expm(skew(w_zn)*th_zn);
% plot
SO3(R_zn).plot
hold on
plot3(vn(1),vn(2),vn(3),'ro')
