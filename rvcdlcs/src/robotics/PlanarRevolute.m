% - Planar Revolute Robot 2D/3D Skeleton Model class (SE3, rpy, stdDH)
% (last mod.: 01-04-2020, Author: Chu Wu)
% Requires rvc & rte https://github.com/star2dust/Robotics-Toolbox
% Properties:
% - name: str (pr*)
% - type: str (elbowup* or elbowdown)
% - link: length of each link (1xm)
% - qlim: limit of each joint (2xm)
% - base: base frame (SE3)
% - tool: tool frame (SE3)
% - height: height above the amounted surface (1x1)
% - twist: twist of each joint (6xm)
% - g_sl0: initial pose of all links (1xm cell of 4x4 matrix)
% - g_st0: initial pose of tool frame (4x4)
% - radius: radius of platform (1x1)
% - altitude: height above the ground (1x1)
% Methods:
% - PlanarRevolute: construction (arg: link) 
% (opt: name, type, base, tool, height, radius ,altitude) 
% - plot (qa: 1xm, qb: 1x6 qrpy/ 1x3 q)
% (opt: workspace, dim, [no]arm, [no]plat, [no]frame, plstyle, 
%       plcolor, plthick, frcolor, frlength, frthick, frstyle, 
%       hgstyle, hgsize, hgcolor, lkcolor, lkthick, lkstyle)
% - animate (qa: 1xm, qb: 1x6 qrpy/ 1x3 q)
% Methods (Static): (for mR manipulator)
% - getTwist: calculate twists by link length and height
% - getFkine: forward kinematics
% - getIkine3: inverse kinematics for 2/3 dof manipulator
% - getJacob: Jacobian
% - getMu: get manipulablity and its derivative
classdef PlanarRevolute < handle
    properties
        % basic
        name
        type
        qlim
        % arm
        link
        base
        tool
        height
        twist
        g_sl0
        g_st0
        % platform
        radius
        altitude
    end
    
    methods
        function obj = PlanarRevolute(varargin)
            % PR.PlanarRevolute  Create m-dof Planar Revolute robot object
            
            % opt statement
            opt.type = 'elbowdown';
            opt.name = 'pr';
            opt.base = SE3;
            opt.tool = SE3;
            opt.height = 1;
            opt.radius = 0.5;
            opt.altitude = 0.5;
            % opt parse: only stated fields are chosen to opt, otherwise to arg
            [opt,arg] = tb_optparse(opt, varargin);
            % check validity
            if length(arg)==1
                link = arg{1}(:)';
            else
                error('unknown argument')
            end
            % type
            m = length(link);
            switch opt.type
                case 'elbowdown'
                    qlim = [zeros(m,1),ones(m,1)*pi/2]';
                case 'elbowup'
                    qlim = [-ones(m,1)*pi/2,zeros(m,1)]';
                otherwise
                    error('Invalid type');
            end
            % struct
            obj.name = opt.name;
            obj.type = opt.type;
            obj.qlim = qlim;
            obj.radius = opt.radius;
            obj.altitude = opt.altitude;
            % choose properties according to type of joints
            import PlanarRevolute.*
            obj.link = link;
            obj.base = opt.base;
            obj.tool = opt.tool;
            obj.height = opt.height;
            [obj.twist, obj.g_sl0 , obj.g_st0] = getTwist(obj.link,opt.height,opt.tool);  
        end
        
        function h = plot(obj,varargin)
            % MPR.plot  Plot m-dof MobileRevolute robot object
            
            % opt statement
            opt.workspace = [];
            opt.dim = 3;
            opt.arm = true;
            opt.plat = false;
            opt.frame = false;
            % platform opt
            opt.plcolor = 'b';
            opt.plthick = 1;
            % frame opt
            opt.frcolor = 'b';
            opt.frlength = 0.5;
            opt.frthick = 0.5;
            opt.frstyle = '-';
            % hinge opt
            opt.hgstyle = 'o';
            opt.hgsize = 5;
            opt.hgcolor = 'b';
            % link opt
            opt.lkcolor = 'b';
            opt.lkthick = 3;
            opt.lkstyle = '-';
            % opt parse: only stated fields are chosen to opt, otherwise to arg
            [opt,arg] = tb_optparse(opt, varargin);
            % argument parse
            if length(arg)==2
                % get pose
                qa = arg{1}(:)';
                % get base
                qb = arg{2}(:)';
            else
                error('unknown arguments');
            end
            if strcmp(get(gca,'Tag'), 'RTB.plot')
                % this axis is an RTB plot window
                rhandles = findobj('Tag', obj.name);
                if isempty(rhandles)
                    % this robot doesnt exist here, create it or add it
                    if ishold
                        % hold is on, add the robot, don't change the floor
                        h = createRobot(obj, qa, qb, opt);
                        % tag one of the graphical handles with the robot name and hang
                        % the handle structure off it
                        %                 set(handle.joint(1), 'Tag', robot.name);
                        %                 set(handle.joint(1), 'UserData', handle);
                    else
                        % create the robot
                        newplot();
                        h = createRobot(obj, qa, qb, opt);
                        set(gca, 'Tag', 'RTB.plot');
                    end
                end
            else
                % this axis never had a robot drawn in it before, let's use it
                h = createRobot(obj, qa, qb, opt);
                set(gca, 'Tag', 'RTB.plot');
                set(gcf, 'Units', 'Normalized');
                %         pf = get(gcf, 'Position');
                %         if strcmp( get(gcf, 'WindowStyle'), 'docked') == 0
                %             set(gcf, 'Position', [0.1 1-pf(4) pf(3) pf(4)]);
                %         end
            end
            view(opt.dim); grid on; rotate3d on
            obj.animate(qa, qb, h.group);
        end
        
        function animate(obj, qa, qb, handles)
            % MPR.animate  Animate m-dof MobileRevolute robot object
            
            if nargin < 4
                handles = findobj('Tag', obj.name);
            end
            % animate
            fk = obj.fkine(qa,qb); p_hg = fk.tv;
            for i=1:length(handles.Children) % draw frame first otherwise there will be delay
                if strcmp(get(handles.Children(i),'Tag'), [obj.name '-tool'])
                    set(handles.Children(i),'matrix',fk(end).T);
                end
                if strcmp(get(handles.Children(i),'Tag'), [obj.name '-base'])
                    set(handles.Children(i),'matrix',fk(i).T);
                end
                if strcmp(get(handles.Children(i),'Tag'), [obj.name '-link'])
                    set(handles.Children(i), 'XData', p_hg(1,:),'YData', p_hg(2,:),'ZData', p_hg(3,:));
                end
                if strcmp(get(handles.Children(i),'Tag'), [obj.name '-plat'])
                    p_fv0 = handles.Children(i).UserData;
                    p_fv = h2e(SE3.qrpy(qb).T*e2h(p_fv0));
                    set(handles.Children(i), 'XData', p_fv(1,:),'YData', p_fv(2,:),'ZData', p_fv(3,:));
                end
            end
        end
        
        function fk = fkine(obj,qa,qb)
            % PR.fkine Forward kinematics for m-dof planar revolute manipulator
            % - qa: joint posture (1xm)
            % - qb: base frame pose (1x6 qrpy/ 1x3 q)
            % - fk: poses of all joints and tool frame (1xm SE3) 
            % use fk(end).t to get the coordinate of tool point
            import PlanarRevolute.*
            for j=1:length(qa)
                g_sl{j} = transl([0,0,obj.height])*obj.g_sl0{j};
                for k=j:-1:1
                    g_sl{j} =  expm(wedge(obj.twist(:,k)).*qa(k))*g_sl{j};
                end
            end
            
            g_ss = {eye(4),transl([0,0,obj.height])}; g_st = obj.g_st0;
            for k=length(qa):-1:1
                g_st =  expm(wedge(obj.twist(:,k)).*qa(k))*g_st;
            end
            
            g_hg = [g_ss,g_sl,{g_st}];
            for i = 1:length(g_hg)
                fk(i) = SE3.qrpy(qb)*SE3([0,0,obj.altitude])*obj.base*SE3(g_hg{i});
            end
        end
        
        function qa = ikine(obj,gfk,qb)
            % PR.ikine Inverse kinematics for m-dof planar revolute manipulator
            % - gfk: SE3 config of forward kinematics (SE3)
            % - qa: joint posture (1xm)
            % - qb: base frame pose (1x6 qrpy/ 1x3 q)
            import PlanarRevolute.*
            rfk = obj.base^-1*SE3([0,0,obj.altitude])^-1*SE3.qrpy(qb)^-1*gfk*obj.tool^-1;
            qtool = SE23(rfk.toqrpy);
%             qtool = rfk.toqrpy;
%             qtool = qtool([1:2,6]);
            if length(obj.link)<=2
                qa = getIkine3(obj.link,qtool,obj.type);
            else
                m = length(obj.link);
                negmup2 = @(th) -det(getJacob(obj.link,th)*getJacob(obj.link,th)');
                qa = fmincon(negmup2,zeros(1,m),[],[],[],[],obj.qlim(1,:),obj.qlim(2,:),@(th) fkcon(obj.link,th,qtool(1:2)));
            end
        end
    end
    
    methods (Access = protected)
        function h = createRobot(obj, qa, qb, opt)
            % create an axis
            ish = ishold();
            if ~ishold
                % if hold is off, set the axis dimensions
                if ~isempty(opt.workspace)
                    axis(opt.workspace);
                end
                hold on
            end
            
            group = hggroup('Tag', obj.name);
            h.group = group;
            
            qb = SE3.qrpy(qb).toqrpy;
            fk = fkine(obj,qa,qb);
            p_hg = fk.tv;
            
            if opt.arm
                h.link = line(p_hg(1,:),p_hg(2,:),p_hg(3,:),'Color',opt.lkcolor,'LineStyle', opt.lkstyle, 'LineWidth', opt.lkthick, 'MarkerFaceColor', opt.hgcolor, 'Marker', opt.hgstyle, 'MarkerSize', opt.hgsize, 'parent', group);
                set(h.link,'Tag', [obj.name '-link']);
            end
            
            if opt.plat
                [X,Y,Z] = cylinder(obj.radius,20);          
                Z = Z*obj.altitude;
                if opt.dim == 2
                    p_fv0 = [X(2,:);Y(2,:);Z(2,:)];
                else                    
                    p_fv0 = [[X(1,:);Y(1,:);Z(1,:)],flip([X(:)';Y(:)';Z(:)'],2),[X(2,:);Y(2,:);Z(2,:)]];                 
                end
                p_fv = h2e(SE3.qrpy(qb).T*e2h(p_fv0));
                h.plat = line(p_fv(1,:),p_fv(2,:),p_fv(3,:),'Color',opt.plcolor, 'LineWidth', opt.plthick, 'parent', group);
                h.plat.UserData = p_fv0;
                set(h.plat,'Tag', [obj.name '-plat']);
            end
            
            if opt.frame
                if opt.arm
                    ftool = fk(end);
                    h.ftool = ftool.plot('color', opt.frcolor,'length',opt.frlength, 'thick', opt.frthick, 'style', opt.frstyle);
                    set(h.ftool,'parent',group);
                    set(h.ftool,'Tag', [obj.name '-tool']);
                end
                if opt.plat
                    fbase = fk(1);
                    h.fbase = fbase.plot('color', opt.frcolor,'length',opt.frlength, 'thick', opt.frthick, 'style', opt.frstyle);
                    set(h.fbase,'parent',group);
                    set(h.fbase,'Tag', [obj.name '-base']);
                end
            end
            
            % restore hold setting
            if ~ish
                hold off
            end
        end
        
%         function updateProps(obj,opt)
%             import PlanarRevolute.*
%             obj.base = opt.base;
%             obj.tool = opt.tool;
%             obj.height = opt.height;
%             [obj.twist, obj.g_sl0 , obj.g_st0] = getTwist(obj.link,opt.height,opt.tool);
%         end
    end
    
    methods (Static)
        function [xi, g_sl0, g_st0] = getTwist(link,height,tool)
            % MPR.getTwist  Calculate POE twist (6 x m+1) by link for mR manipulator
            % - link: link lengths (1xm)
            % - height: height above the surface of base (1x1)
            % - tool: transformation from the last joint to tool frame
            
            % mounted place (translation only for mR)
            % indeed hb_T can be chosen as any transformation
            % mounted place (translation only)
            height_T = transl([0,0,height]);
            l0 = [0;link(:)];
            for i=1:length(l0)
                p_hg0(:,i) = [sum(l0(1:i)),0,0]';
            end
            for i=1:length(link)
                g_sl0{i} = transl([sum(link(1:i)),0,0]);
            end
            % rotation axis
            w = [0,0,1]';
            % joint twists
            for i=1:length(l0)-1
                xi(:,i) = [-skew(w)*p_hg0(:,i);w];
            end
            % tool twist
            g_st0 = height_T*transl(p_hg0(:,end))*tool.T;
            xi(:,i+1) = vee(logm(g_st0));
        end
        
        
        function pfk = getFkine(link,th)
            % PR.getFkine  Forward kinematics coordinate for m-dof planar revolute manipulator
            % - link: link lengths (1xm)
            % - th: joint angles (1xm)
            % - pfk: position of end-effector (1x2)
            
            m = length(th);
            ls = zeros(m,1);
            lc = ls;
            % lsin and lcos
            pfk = [0;0];
            for i=1:m
                ls(i) = link(i)*sin(sum(th(1:i)));
                lc(i) = link(i)*cos(sum(th(1:i)));
                pfk = pfk+[lc(i);ls(i)];
            end
            pfk = pfk(:)';
        end
        
        function th = getIkine3(link,qfk,type)
            % PR.prIkine  Inverse kinematics for 2 or 3-dof planar revolute manipulator
            % - link: link lengths (1xm)
            % - qfk: pose of forward kinematics (1x3) (SE2 configuration [x,y,phi])
            % - type: elbow type ('elbowup' or 'elbowdown')
            
            if length(link)==3
                l1 = link(1); l2 = link(2); l3 = link(3);
                end_SE3 = SE2(qfk)*SE2([-l3,0]);
            elseif length(link)==2
                l1 = link(1); l2 = link(2);
                end_SE3 = SE2(qfk);
            else
                error('link length exceeds the degree of freedom (max 3)');
            end
            gT = end_SE3.q;
            x = gT(1); y = gT(2); thT = gT(3);
            % angle => inverse kinematics of three-link planar robot
            if nargin<3
                type = 'elbowdown';
            end
            switch type
                case 'elbowup' % up-elbow
                    ue = -1;
                case 'elbowdown' % down-elbow
                    ue = 1;
                otherwise
                    error('invalid elbow type');
            end
            c2 = (x^2+y^2-l1^2-l2^2)/(2*l1*l2);
            th2 = acos(c2)*ue;
            s2 = sqrt(1-c2^2);
            th1 = atan2(y,x)-atan2(l2*s2,l1+l2*c2)*ue;
            if length(link)==3
                th3 = thT-th1-th2;
                th = [th1,th2,th3];
            else
                th = [th1,th2];
            end
        end
        
        function [mu,dmu] = getMu(link,th)
            % PR.getMu   Calculate manipulability and its derivative
            % - th: joint angles (1xm)
            % - mu: manipulability (1x1)
            % - dmu: derivative of manipulability(1xm)

            import PlanarRevolute.*
            m = length(link);
            % Jacobian
            J = getJacob(link,th);
            dx = J(1,:)'; dy = J(2,:)';
            % mu^2
            mu = sqrt(det(J*J'));
            % second method to calcuate row vec of Hessian
            Hdy = kron(ones(1,m),dy);
            Hdx = kron(ones(1,m),dx);
            for i=1:m-1
                for j=i:m
                    Hdy(i,j) = dy(j);
                    Hdx(i,j) = dx(j);
                end
            end
            rvecH = [-Hdy,Hdx];
            % nabla mu
            JJTinv = (J*J')^-1;
            dmu = mu*rvecH*kron(eye(2),J')*JJTinv(:);
        end
        
        function J = getJacob(link,th)
            % PR.getJacob  Jacobian for m-dof planar revolute manipulator
            % - link: link lengths (1xm)
            % - th: joint angles (1xm)
            
            m = length(th);
            ls = zeros(m,1);
            lc = ls; dx = ls; dy = ls;
            % lsin and lcos
            for i=1:m
                ls(i) = link(i)*sin(sum(th(1:i)));
                lc(i) = link(i)*cos(sum(th(1:i)));
            end
            % dxdth and dydth
            for i=1:m
                dx(i) = 0;
                dy(i) = 0;
                for j=i:m
                    dx(i) = dx(i)-ls(j);
                    dy(i) = dy(i)+lc(j);
                end
            end
            % Jacobian
            J = [dx,dy]';
        end
        
        function [c,ceq] = fkcon(link,th,pfk)
            % PR.fkcon  Get the forward kinematic equation constraints
            % - link: link lengths (1xm)
            % - th: joint angles (1xm)
            % - pfk: desired position of end-effector (1x2)
            import PlanarRevolute.*
            ceq = getFkine(link,th)-pfk(:)';
            c = [];
        end
    end
end