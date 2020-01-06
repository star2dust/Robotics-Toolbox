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
% Methods (Static): (for mR manipulator)
% - mrtwist: calculate twists by link length and mounted base
% - mrfkine: translation forward kinematics and Jacobian
% - mrmanipulability: manipulability and its derivative
classdef CooperativeManipulation < handle
    properties
        name
        robot
        payload
        height
    end
    
    methods
        function obj = CooperativeManipulation(varargin)
            % MobileRevolute creates m-dof MobileRevolute robot object
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
        
        function h = plot(varargin)  
            % opt statement
            opt.workspace = [];
            opt.frame = false;
            opt.framecolor = 'b';
            % opt parse: only stated fields are chosen to opt, otherwise to arg
            [opt,arg] = tb_optparse(opt, varargin); 
            % argument parse
            if length(arg)==3
                % get object
                obj = arg{1};
                % get pose
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
                        h = createCM(obj, qe, qr, opt);
                        % tag one of the graphical handles with the robot name and hang
                        % the handle structure off it
                        %                 set(handle.joint(1), 'Tag', robot.name);
                        %                 set(handle.joint(1), 'UserData', handle);
                    else
                        % create the robot 
                        newplot();
                        h = createCM(obj, qe, qr, opt);
                        set(gca, 'Tag', 'RTB.plot');
                    end 
                end      
            else
                % this axis never had a robot drawn in it before, let's use it
                h = createCM(obj, qe, qr, opt);
                set(gca, 'Tag', 'RTB.plot');
                set(gcf, 'Units', 'Normalized');
                pf = get(gcf, 'Position');
                %         if strcmp( get(gcf, 'WindowStyle'), 'docked') == 0
                %             set(gcf, 'Position', [0.1 1-pf(4) pf(3) pf(4)]);
                %         end
            end
            view(3); grid on;
            obj.animate(qe,qr);         
        end 
        
        function animate(obj,qe,qr)
            if nargin < 4
                handles = findobj('Tag', obj.name);
            end
            [qe,qr] = obj.min2config(qe,qr);
            % animate
            for i=1:length(obj.robot)
                obj.robot(i).animate(qr(i,:));
            end
            qc = sum(qe,1)/size(qe,1);
            obj.payload.animate(qc);
        end
    end
        
    methods (Access = protected) 
        function h = createCM(obj, qe, qr, opt)
            % create an axis
            ish = ishold();
            if ~ishold
                % if hold is off, set the axis dimensions
                if ~isempty(opt.workspace)
                    axis(opt.workspace);
                end
                hold on
            end
            
            [qe,qr] = obj.min2config(qe,qr);
            
            group = hggroup('Tag', obj.name);
            h.group = group;
            qc = sum(qe,1)/size(qe,1);
            if opt.frame
                h.payload = obj.payload.plot(qc,'facecolor','g','frame','framecolor', opt.framecolor);
                for i=1:length(obj.robot)
                    h.robot(i) = obj.robot(i).plot(qr(i,:),'frame','framecolor', opt.framecolor);
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
        
        function [qqe,qqr] = min2config(obj,qe,qr)
            if size(qe,2)==3
                qe_flag = true;
            end
            if size(qr,2)==obj.robot(1).arm.n-1
                qr_flag = true;
            end
            for i=1:length(obj.robot)
                % qe: 3=>6, qr: m-1 => 3+m
                if qe_flag
                    qqe(i,1:6) = [qe(i,1:2),obj.height,0,0,qe(i,3)];
                end
                if qr_flag
                    qa(i,:) = [qe(i,3)-sum(qr(i,:)),qr(i,:)]; % m-dof arm
                    f(i,:) = obj.robot(i).mrfkine(qa(i,:),obj.robot(i).link)';
                    qqr(i,1:3+obj.robot(i).arm.n) = [qe(i,1:2)-f(i,:),0,qa(i,:)];
                end
            end
        end
    end
end