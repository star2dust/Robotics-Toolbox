function xi_mat = dh2poe(dh_mat,type,H_base,H_tool)
% Conversion from standar DH parameters to POE parameters
% xi: 6 x m+1 => base to [link 0, ..., link m, tool]
xi_R = [0,0,0,0,0,1]';
xi_P = [0,0,1,0,0,0]';
if nargin>2
    gl0_sdh{1} = H_base; % g_{l_{i-2}l_{i-1}}(0) (base to link 0, i.e. j-1 = 0 in gl0_sdh{j})
else
    gl0_sdh{1} = eye(4);
end
for i=1:size(dh_mat,1)
    gl0_sdh{i+1} = stdDH(dh_mat(i,:));
    xi_mat(:,i) = (type(i)=='R')*xi_R+(type(i)=='P')*xi_P;
    for j=i:-1:1
        xi_mat(:,i) = Adg(gl0_sdh{j})*xi_mat(:,i);
    end
end
g_st = gl0_sdh{1};
for i=2:length(gl0_sdh)
    g_st = g_st*gl0_sdh{i};
end
if nargin>3
    g_st = g_st*H_tool; % link m to tool
end
xi_mat(:,end+1) = vee(logm(g_st));
end