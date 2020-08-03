% - Mobile Robot 3D Model class (SE3, rpy, stdDH)
% (last mod.: 29-07-2020, Author: Chu Wu)
% Requires rvc & rte https://github.com/star2dust/Robotics-Toolbox
% Properties:
% - name: str (robot*)
% - base: Platform
% - arm: SerialLink
% - mount: SE3 (on the upper surface of mobile base)
% Methods:
% - MobileRobot: construction (opt: name, qlim)
% - plot (opt: workspace, [no]frame, framecolor)
% - animate
classdef MobileRobot < handle
    properties
        name
        base % Platform
        arm % SerialLink
        mount % SE3
    end
    
    methods
        function obj = MobileRobot(varargin)
            % MR.MobileRobot   Create MobileRobot robot object
            
            % opt statement
            opt.name = 'robot';
            opt.qlim = [];
            % opt parse: only stated fields are chosen to opt, otherwise to arg
            [opt,arg] = tb_optparse(opt, varargin); 
            obj.name = opt.name; 
            % argument parse
            if length(arg)==3
                edge = arg{1}(:)';
                twist = arg{2};
                Hb = arg{3};
                [dh, Ht, sigma] = poe2dh(twist); 
                dh = [dh,sigma];
                obj.mount = SE3(Hb);              
            elseif length(arg)==4
                edge = arg{1}(:)';
                dh = arg{2};
                Hb = arg{3};
                Ht = arg{4};
                obj.mount = SE3(Hb);         
            else
                error('unknown arguments');
            end
            plotopt = {'noname', 'nobase', 'notiles', 'noshading', 'noshadow', 'nowrist'};
            % set qlim
            if ~isempty(opt.qlim)
                obj.base = Platform(edge,'name', [obj.name '-base']);
                obj.arm = SerialLink(dh, 'name', [obj.name '-arm'], 'base', Hb, 'tool', Ht, 'plotopt', plotopt, 'qlim', opt.qlim);
            else
                obj.base = Platform(edge,'name', [obj.name '-base']);
                obj.arm = SerialLink(dh, 'name', [obj.name '-arm'], 'base', Hb, 'tool', Ht, 'plotopt', plotopt);
            end
        end
        
        function h = plot(obj,varargin) 
            % MR.plot   Plot MobileRobot robot object
            
            % opt statement
            opt.workspace = [];
            opt.frame = false;
            opt.framecolor = 'b';
            opt.framelength = sum(obj.base.edge)/length(obj.base.edge)/3;
            opt.framethick = 1;
            opt.framestyle = '-';
            % opt parse: only stated fields are chosen to opt, otherwise to arg
            [opt,arg] = tb_optparse(opt, varargin); 
            % argument parse
            if length(arg)==1
                % get pose
                q = arg{1}(:)';
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
                        h = createRobot(obj, q, opt);
                        % tag one of the graphical handles with the robot name and hang
                        % the handle structure off it
                        %                 set(handle.joint(1), 'Tag', robot.name);
                        %                 set(handle.joint(1), 'UserData', handle);
                    else
                        % create the robot 
                        newplot();
                        h = createRobot(obj, q, opt);
                        set(gca, 'Tag', 'RTB.plot');
                    end 
                end      
            else
                % this axis never had a robot drawn in it before, let's use it
                h = createRobot(obj, q, opt);
                set(gca, 'Tag', 'RTB.plot');
                set(gcf, 'Units', 'Normalized');
                pf = get(gcf, 'Position');
                %         if strcmp( get(gcf, 'WindowStyle'), 'docked') == 0
                %             set(gcf, 'Position', [0.1 1-pf(4) pf(3) pf(4)]);
                %         end
            end
            view(3); grid on;
            obj.animate(q,h.group);
        end 
        
        function animate(obj,q,handles)
            % MR.animate  Animate MobileRobot robot object
            if nargin < 3
                handles = findobj('Tag', obj.name);
            end
            % update pose
            qa = q(7:end); qb = q(1:6);
            obj.arm.base = SE3.qrpy(qb)*obj.mount;
            % animate
            for i=1:length(handles.Children) % draw frame first otherwise there will be delay
                if strcmp(get(handles.Children(i),'Tag'), [obj.name '-tool'])
                    set(handles.Children(i),'matrix',obj.arm.fkine(qa).T);
                end
                if strcmp(get(handles.Children(i),'Tag'), [obj.name '-base'])
                    obj.base.animate(qb,handles.Children(i));
                end
                if strcmp(get(handles.Children(i),'Tag'), [obj.name '-arm'])
                    obj.arm.animate(qa,handles.Children(i));
                end
            end 
        end
    end
    
    methods (Access = protected)   
        function h = createRobot(obj, q, opt)
            % create an axis
            ish = ishold();
            if ~ishold
                % if hold is off, set the axis dimensions
                if ~isempty(opt.workspace)
                    axis(opt.workspace);
                end
                hold on
            end
            % update pose
            qa = q(7:end); qb = q(1:6);
            obj.arm.base = SE3.qrpy(qb)*obj.mount;
            
            group = hggroup('Tag', obj.name);
            h.group = group;
            h.arm = obj.arm.plot(qa);
            if opt.frame
                h.base = obj.base.plot(qb,'frame','framecolor', opt.framecolor,'framelength',opt.framelength, 'framethick', opt.framethick, 'framestyle', opt.framestyle);
                h.tool = SE3(obj.arm.fkine(qa)).plot('color', opt.framecolor,'length',opt.framelength, 'thick', opt.framethick, 'style', opt.framestyle);
                set(h.tool,'parent',group);
                set(h.tool,'Tag', [obj.name '-tool']);
            else
                h.base = obj.base.plot(qb);
            end
            set(h.arm.group,'parent',group);
            set(h.base.group,'parent',group);
                
            % restore hold setting
            if ~ish
                hold off
            end
        end
    end
end