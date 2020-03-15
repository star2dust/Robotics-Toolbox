%% intersection constraints
for i=1:4
    % linear programming [X;Y]=f*[g_ce(1:2);z]
    R1{i} = rot2(g_ce_lb(3,i))'; f{i} = [R1{i},-R1{i}*D{i}]';
    [~,X_lb(:,i)] = linprog(f{i}(:,1),[],[],[],[],[g_ce_lb(1:2,i);z_lb(:,i)],[g_ce_ub(1:2,i);z_ub(:,i)]);
    [~,X_ub(:,i)] = linprog(-f{i}(:,1),[],[],[],[],[g_ce_lb(1:2,i);z_lb(:,i)],[g_ce_ub(1:2,i);z_ub(:,i)]);
    [~,Y_lb(:,i)] = linprog(f{i}(:,2),[],[],[],[],[g_ce_lb(1:2,i);z_lb(:,i)],[g_ce_ub(1:2,i);z_ub(:,i)]);
    [~,Y_ub(:,i)] = linprog(-f{i}(:,2),[],[],[],[],[g_ce_lb(1:2,i);z_lb(:,i)],[g_ce_ub(1:2,i);z_ub(:,i)]);
    % initials
    X0(i) = f{i}(:,1)'*[g_ce0(1:2,i);z0(:,i)];
    Y0(i) = f{i}(:,2)'*[g_ce0(1:2,i);z0(:,i)];
    TH0 = th_tilde0;
    % modify joint space to convex space 
    J1{i} = [-la(1)*sin(g_ce0(3,i)-TH0(i))-la(2)*sin(g_ce0(3,i)), la(1)*sin(g_ce0(3,i)-TH0(i));
        la(1)*cos(g_ce0(3,i)-TH0(i))+la(2)*cos(g_ce0(3,i)),-la(1)*cos(g_ce0(3,i)-TH0(i))];
end
X_ub = -X_ub; Y_ub = -Y_ub;
TH_lb = th_tilde_lb; TH_ub = th_tilde_ub;
% plot original/modified joint space
vis_jotspc_on = 0; vis_org = 0;
if vis_jotspc_on
    for i=1:4
        figure
        [X,Y,TH] = meshgrid(X_lb(:,i)-.01:.01:X_ub(:,i)+.01,Y_lb(:,i)-.01:.01:Y_ub(:,i)+.01,TH_lb(:,i)-.01:.01:TH_ub(:,i)+.01);
        cx = max(-X+X_lb(:,i),X-X_ub(:,i));
        cy = max(-Y+Y_lb(:,i),Y-Y_ub(:,i));
        cz = max(-TH+TH_lb(:,i),TH-TH_ub(:,i));
        cth = max(g_ce0(3,i)-g_ce_lb(3,i)-TH-pi/2,-(g_ce0(3,i)-g_ce_lb(3,i)-TH));
        if vis_org
            Fkx{i} = R1{i}(1,1)*(la(1)*cos(g_ce0(3,i)-TH)+la(2)*cos(g_ce0(3,i)))+R1{i}(1,2)*(la(1)*sin(g_ce0(3,i)-TH)+la(2)*sin(g_ce0(3,i)));
            Fky{i} = R1{i}(2,1)*(la(1)*cos(g_ce0(3,i)-TH)+la(2)*cos(g_ce0(3,i)))+R1{i}(2,2)*(la(1)*sin(g_ce0(3,i)-TH)+la(2)*sin(g_ce0(3,i)));
            ints = max(-(X-Fkx{i}),-(Y-Fky{i}));
        else            
            % tangent plane
            intA0 = [eye(2),-R1{i}*J1{i}]; intb0 = intA0*[X0(i);Y0(i);g_ce0(3,i);TH0(i)];
            ints = max(-intA0(1,1)*X-intA0(1,2)*Y-intA0(1,3)*g_ce0(3,i)-intA0(1,4)*TH+intb0(1),-intA0(2,1)*X-intA0(2,2)*Y-intA0(2,3)*g_ce0(3,i)-intA0(2,4)*TH+intb0(2));
        end
        cons = max(ints,max(cx,max(cy,max(cz,cth))));
        facevert = isosurface(X,Y,TH,cons,0);
        pat = patch(facevert);
        isonormals(X,Y,TH,cons,pat)
        set(pat,'facecolor',[0 .5 1],'edgecolor','none');hold on
        view(150,30),axis image,grid on
        ylabel('Y');xlabel('X');zlabel('TH');
        camlight
        lighting gouraud
        plot3(X0(i),Y0(i),TH0(i),'ro');
    end
end