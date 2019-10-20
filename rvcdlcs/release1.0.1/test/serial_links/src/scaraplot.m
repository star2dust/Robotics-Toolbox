function link_han = scaraplot(rob_struct,th_mat,axis_vec)
% plot scara robot by joint angles
% parameters
l_vec = [rob_struct.link.length];
xi_cell = {rob_struct.link.twist};
g_sl0 = [rob_struct.link.refconfig]; 
g_ps = rob_struct.base;
l1 = l_vec(1); l2 = l_vec(2); l3 = l_vec(3); l4 = l_vec(4);
xi = xi_cell; thtraj = th_mat;
% plot scara robot
gcf;
view(-38,28); hold on
axis(axis_vec)
% links
rod_x = [[0,0,0]',[1,0,0]']-[0.5,0,0]';
rod_z = roty(pi/2)*rod_x;
rod_x1 = rod_x*l1; rod_x2 = rod_x*l2; 
rod_z3 = rod_z*l3; rod_x4 = rod_x*l4;
rod_z0 = (rod_z+[0,0,0.5]')*l3;
% set position
ldata1 = g_ps*g_sl0(1)*rod_x1;
ldata2 = g_ps*g_sl0(2)*rod_x2;
ldata3 = g_ps*g_sl0(3)*rod_z3;
ldata4 = g_ps*g_sl0(4)*rod_x4;
% plot
link_han(1) = plot3(rod_z0(1,:),rod_z0(2,:),rod_z0(3,:),'k');
link_han(2) = plot3(ldata1(1,:),ldata1(2,:),ldata1(3,:),'r');
link_han(3) = plot3(ldata2(1,:),ldata2(2,:),ldata2(3,:),'m');
link_han(4) = plot3(ldata3(1,:),ldata3(2,:),ldata3(3,:),'b');
link_han(5) = plot3(ldata4(1,:),ldata4(2,:),ldata4(3,:),'g');
hold off
%% robot animate
for i=1:size(thtraj,1)
    for j=1:size(thtraj,2)
        g_sl(j) = g_sl0(j);
        for k=j:-1:1
            g_sl(j) =  SE3(expm(hatwedge(xi{k}).*thtraj(i,k)))*g_sl(j);
        end
    end
    % update position
    ldata1 = g_ps*g_sl(1)*rod_x1;
    ldata2 = g_ps*g_sl(2)*rod_x2;
    ldata3 = g_ps*g_sl(3)*rod_z3;
    ldata4 = g_ps*g_sl(4)*rod_x4;
    set(link_han(2),'XData',ldata1(1,:),'YData',ldata1(2,:),'ZData',ldata1(3,:));
    set(link_han(3),'XData',ldata2(1,:),'YData',ldata2(2,:),'ZData',ldata2(3,:));
    set(link_han(4),'XData',ldata3(1,:),'YData',ldata3(2,:),'ZData',ldata3(3,:));
    set(link_han(5),'XData',ldata4(1,:),'YData',ldata4(2,:),'ZData',ldata4(3,:));
    drawnow
end
end