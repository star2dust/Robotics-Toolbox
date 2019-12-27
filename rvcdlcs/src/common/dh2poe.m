function xi_mat = dh2poe(dh_mat,type)
import SE3.*
xi_R = [0,0,0,0,0,1]';
xi_P = [0,0,1,0,0,0]';
gl_sdh{1} = eye(4); % gl_{i-2}{i-1} (base to link 0)
for i=1:size(dh_mat,1)
    gl_sdh{i+1} = stdDH(dh_mat(i,:));
    xi_mat(:,i) = (type(i)=='R')*xi_R+(type(i)=='P')*xi_P;
    for j=i:-1:1
        xi_mat(:,i) = Adg(gl_sdh{j})*xi_mat(:,i);
    end
end
g_st = gl_sdh{1};
for i=2:length(gl_sdh)
    g_st = g_st*gl_sdh{i};
end
xi_mat(:,end+1) = hatvee(logm(g_st));
end