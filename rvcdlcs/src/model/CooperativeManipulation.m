% Cooperative Manipulation 3D Model class (rpy, stdDH)
% (last mod.: 06-01-2020, Author: Chu Wu)
% Requires rvc & rte https://github.com/star2dust/Robotics-Toolbox
% Properties:
% - name: str (cm*)
% - robot: MobilePlanarRevolute
% - payload: Cylinder
% - height: payload height above the ground
% Methods:
% - CooperativeManipulation: construction (opt: name) (arg: edge, link, mh, radius)
% - plot (opt: workspace, [no]frame, framecolor)
% - animate
classdef CooperativeManipulation < handle
    properties
        name
        robot
        payload
        height
    end
    
    methods
        function obj = CooperativeManipulation(varargin)
            % CM.CooperativeManipulation   Creates planar cooperative manipulation object
            % opt statement
            opt.name = 'cm';
            % opt parse: only stated fields are chosen to opt, otherwise to arg
            [opt,arg] = tb_optparse(opt, varargin);
            % check validity
            if length(arg)==3
                nrob = arg{1};
                nlink = arg{2};
                radius = arg{3};
            else
                error('unknown argument')
            end
            link = ones(nlink,1)*0.8*radius; edge = [1,.8,.5]*radius; mh = [0,0,1]*radius;
            % construct robot
            obj.robot = MobilePlanarRevolute(edge,link,mh,'name','rob1','type','elbowup');
            for i=2:nrob
                obj.robot(i) = MobilePlanarRevolute(edge,link,mh,'name',['rob' num2str(i)],'type','elbowup');
            end
            obj.payload = Cylinder(radius,0.2,'name','payload');
            obj.name = opt.name;
            obj.height = edge(3)+mh(3);
        end
        
        function h = plot(obj,varargin)
            % opt statement
            opt.workspace = [];
            opt.frame = false;
            opt.framecolor = 'br';
            opt.framelength = obj.payload.radius/3;
            opt.framethick = 1;
            opt.framestyle = '-';
            % opt parse: only stated fields are chosen to opt, otherwise to arg
            [opt,arg] = tb_optparse(opt, varargin);
            % argument parse
            if length(arg)==3
                % get pose
                qc = arg{1};
                qe = arg{2};
                qr = arg{3};
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
                        h = createCM(obj, qc, qe, qr, opt);
                        % tag one of the graphical handles with the robot name and hang
                        % the handle structure off it
                        %                 set(handle.joint(1), 'Tag', robot.name);
                        %                 set(handle.joint(1), 'UserData', handle);
                    else
                        % create the robot
                        newplot();
                        h = createCM(obj, qc, qe, qr, opt);
                        set(gca, 'Tag', 'RTB.plot');
                    end
                end
            else
                % this axis never had a robot drawn in it before, let's use it
                h = createCM(obj, qc, qe, qr, opt);
                set(gca, 'Tag', 'RTB.plot');
                set(gcf, 'Units', 'Normalized');
                pf = get(gcf, 'Position');
                %         if strcmp( get(gcf, 'WindowStyle'), 'docked') == 0
                %             set(gcf, 'Position', [0.1 1-pf(4) pf(3) pf(4)]);
                %         end
            end
            view(3); grid on;
            obj.animate(qc,qe,qr);
        end
        
        function animate(obj,qc,qe,qr)
            if nargin < 4
                handles = findobj('Tag', obj.name);
            end
            [qc,qr] = obj.min2config(qc,qe,qr);
            % animate
            obj.payload.animate(qc);
            for i=1:length(obj.robot)
                obj.robot(i).animate(qr(i,:));
            end   
        end
        
        
        function g_r2m = g_r2m(obj,g_r2e,th_tilde)
            % CM.g_r2m Transfer reference frame to mobile base frame (SE2)
            n = length(obj.robot);
            for i=1:n
                % real mobile base pose
                m = obj.robot(i).arm.n;
                delta = [-ones(1,m-1);eye(m-1)];
                th(i,:) = (delta*th_tilde(i,:)')';
                p_e2m(:,i) = -obj.robot(i).mrfkine(obj.robot(i).link,th(i,:));
                p_r2m(:,i) = h2e(g_r2e(i).T*e2h(p_e2m(:,i)));
                g_r2m(i) = SE2([p_r2m(:,i)',0]);
            end
        end
        
        function [g_r2e,p_r2m_hat,th_r] = gfkine(obj,g_r2c,psi,scale)
            % CM.gfkine Return grasp and formation kinematics (SE2)
            n = length(obj.robot);
            for i=1:n    
                % g_r2e
                phi_c2e_bar(i) = -(n-1)*pi/n+2*pi/n*(i-1);
                p_c2e_bar(:,i) = obj.payload.radius*[-cos(phi_c2e_bar(i)),-sin(phi_c2e_bar(i))];
                p_r2e(:,i) = h2e(g_r2c.T*e2h(p_c2e_bar(:,i)));
                phi_r2e(i) = phi_c2e_bar(i)+psi;
                g_r2e(i) = SE2(p_r2e(:,i),phi_r2e(i));
                % p_r2m_hat
                [~,th_e_bar(i,:)] = obj.robot(i).manipulability;
                p_e2m_bar(:,i) = -obj.robot(i).mrfkine(obj.robot(i).link,th_e_bar(i,:));
                % initial q_r2c and psi are zeros, so p_r_c2m_bar = p_c2m_bar
                p_r_c2m_bar(:,i) = h2e(SE2(p_c2e_bar(:,i),phi_c2e_bar(i)).T*e2h(p_e2m_bar(:,i))); 
                p_r2m_hat(:,i) = g_r2c.t+scale*p_r_c2m_bar(:,i);
                % th_e
                base_r2m = SE3.qrpy([p_r2m_hat(:,i)',obj.robot(i).base.edge(3)/2,0,0,0]);
                tool_r2e = SE3.qrpy([p_r2e(:,i)',obj.height,0,0,phi_r2e(i)]);
                q_r(i,:) = obj.robot(i).ikine(tool_r2e,base_r2m);
                th_r(i,:) = q_r(i,4:end);
            end
        end
    end
    
    methods (Access = protected)
        function h = createCM(obj, qc, qe, qr, opt)
            % create an axis
            ish = ishold();
            if ~ishold
                % if hold is off, set the axis dimensions
                if ~isempty(opt.workspace)
                    axis(opt.workspace);
                end
                hold on
            end
            
            [qc,qr] = obj.min2config(qc,qe,qr);
            
            group = hggroup('Tag', obj.name);
            h.group = group;
            if opt.frame
                if length(opt.framecolor)>1
                    h.payload = obj.payload.plot(qc,'facecolor','g','frame','framecolor', opt.framecolor(2),'framelength', opt.framelength, 'framethick', 2);
                else
                    h.payload = obj.payload.plot(qc,'facecolor','g','frame','framecolor', opt.framecolor(1),'framelength', opt.framelength, 'framethick', 2);
                end   
                for i=1:length(obj.robot)
                    h.robot(i) = obj.robot(i).plot(qr(i,:),'frame','framecolor', opt.framecolor(1),'framelength',opt.framelength, 'framethick', opt.framethick, 'framestyle', opt.framestyle);
                end
            else
                h.payload = obj.payload.plot(qc,'facecolor','g');
                for i=1:length(obj.robot)
                    h.robot(i) = obj.robot(i).plot(qr(i,:));
                end
            end
            set(h.robot(i).group,'parent',group);
            set(h.payload.group,'parent',group);
            
            % restore hold setting
            if ~ish
                hold off
            end
        end
        
        function [qqc,qqr] = min2config(obj,qc,qe,qr)
            % qc: 3=>6, qe: 3=>6, qr: m-1 => 3+m      
            if size(qc,2)==3
                qqc(1:6) = [qc(1:2),obj.height,0,0,qc(3)];
            elseif size(qc,2)==6
                qqc = qc;
            else
                error('invalid argument');
            end
            for i=1:length(obj.robot)
                if size(qe,2)==3
                    qqe(i,1:6) = [qe(i,1:2),obj.height,0,0,qe(i,3)];
                elseif size(qe,2)==6
                    qqe = qe;
                else
                    error('invalid argument');
                end
                if size(qr,2)==obj.robot(1).arm.n-1
                    qa(i,:) = [qqe(i,6)-sum(qr(i,:)),qr(i,:)]; % m-dof arm
                    f(i,:) = obj.robot(i).mrfkine(obj.robot(i).link,qa(i,:))';
                    qqr(i,1:3+obj.robot(i).arm.n) = [qqe(i,1:2)-f(i,:),0,qa(i,:)];
                elseif size(qr,2)==obj.robot(1).arm.n+3
                    qqr = qr;
                else
                    error('invalid argument');
                end
            end
        end
    end
end