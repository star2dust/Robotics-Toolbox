close all
clear

% % obstacle read
% scale = 0.5; cub_edge = [1,1,1]*scale; tile_edge = cub_edge(1:2);
% [map, tile_q, tile_ind] = obs_read('01.png',tile_edge);
% for i=1:length(tile_ind)
%     tile_obs(i) = Cuboid2(tile_edge,'name',['tile' num2str(i)]);
% end
% figure;
% for i=1:length(tile_ind)
%     tile_obs(i).plot(tile_q(i,:),'workspace', [0 tile_edge(1)*size(map,1) 0 tile_edge(2)*size(map,2)],'facecolor','k');
%     hold on;
% end
% view(2)

% generate path
q_c(1,:) = [2.5,2.5,0];
q_c(2,:) = [5,5,pi/2];
q_c(3,:) = [10,10,0];
q_c(4,:) = [15,10,0];
q_c(5,:) = [20,15,pi/4];
q_c(6,:) = [20,20,pi/2];
h_p_c = plot(q_c(:,1),q_c(:,2),'mo');
[q_c,dq_c,ddq_c,tq_c] = calctraj(q_c,0.5*ones(size(q_c(1,:))),0.1,2);
% SE2(q_c).animate

% robot formation
fd = [1,0,-1,0;
    0,1,0,-1];
fs = [1,1,-1,-1;
    -1,1,1,-1];

p_dl = [fd*1,fs*2.5]';
[p_du, i_pu] = convcls(p_dl);
p_dl = p_dl(i_pu,:);
p_dl_conv = p_dl(convhull(p_dl),:);
h_pm = line(p_dl(:,1),p_dl(:,2),'Marker','o','Color','b');
h_pm_conv = line(p_dl_conv(:,1),p_dl_conv(:,2),'LineStyle', '-.','Color','r');

% change scale
slim = [0.5,1.5];
pp_du = [p_du(end,:);p_du];
for i=1:size(p_du,1)-1
   v_ll(i,:) = slim(1)*pp_du(i,:)-slim(2)*p_du(i,:);
   v_lu(i,:) = slim(1)*pp_du(i+2,:)-slim(2)*p_du(i,:);
   v_ul(i,:) = slim(2)*pp_du(i,:)-slim(1)*p_du(i,:);
   v_uu(i,:) = slim(2)*pp_du(i+2,:)-slim(1)*p_du(i,:);
   th_Tll(i) = cart2pol(v_ll(i,1),v_ll(i,2));
   th_Tlu(i) = cart2pol(v_lu(i,1),v_lu(i,2));
   Tl{i} = [skew2(v_ll(i,:))/norm(skew2(v_ll(i,:)));skew2(v_lu(i,:))/norm(skew2(v_lu(i,:)))];
   Tu{i} = [skew2(v_ul(i,:))/norm(skew2(v_ul(i,:)));skew2(v_uu(i,:))/norm(skew2(v_uu(i,:)))];
end
for i=1:size(p_du,1)
    p_circ = circle(p_dl(i,:), 5)';
    h_circ = line(p_circ(:,1),p_circ(:,2),'Color','g');
end

% update 
tic;
playspeed = 2; dt = 0.04;
while toc<tq_c(end)/playspeed
    tk = toc*playspeed;
    % choose via point in time 'tk'
    q_c_tk = interp1(tq_c,q_c,tk); 
    % update object
    p_m = h2e(SE2(q_c_tk).T*e2h(p_dl'))';
    p_m_conv = h2e(SE2(q_c_tk).T*e2h(p_dl_conv'))';
    set(h_pm,'XData',p_m(:,1),'YData',p_m(:,2));
    set(h_pm_conv,'XData',p_m_conv(:,1),'YData',p_m_conv(:,2));
    drawnow
    dt = toc*playspeed - tk;
end
toc



diff([th_Tlu;th_Tll])
