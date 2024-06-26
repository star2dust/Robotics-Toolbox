% Mobile Manipulator 3D Model class (SE3, rpy, stdDH)
% (last mod.: 03-08-2020, Author: Chu Wu)
% Requires rvc & rte https://github.com/star2dust/Robotics-Toolbox
% Properties:
% - name: str (robot*)
% - platform: Platform
% - arm: SerialLink
% - mount: SE3 (on the upper surface of mobile base)
% Methods:
% - MobileRobot: construction (opt: name, qlim)
% - plot (opt: workspace, [no]frame, framecolor)
% - animate
classdef MobileManipulator < SerialLink
    properties
        platform % Platform
        arm % SerialLink
        real3d
    end
    
    methods
        function obj = MobileManipulator(varargin)
            % Create MobileManipulator object
            
            % opt statement
            opt.B = zeros(2,1);
            opt.Tc = zeros(2,2);
            opt.qlim = [];
            opt.name = 'robot';
            opt.real3d = false;
            % opt parse: only stated fields are chosen to opt, otherwise to arg
            [opt,arg] = tb_optparse(opt, varargin); 
            % argument parse
            if length(arg)==3
                edge = arg{1}(:)';
                dh = arg{2};
                Ht = arg{3};         
            else
                error('unknown arguments');
            end
            if size(dh,2)<5
                error('dh should be Nx5 (N>3)');
            end
            % set base
            if opt.real3d
                mmplat = Platform(edge, 'name', 'base', 'B', opt.B, 'Tc', opt.Tc, 'real3d');
            else
                mmplat = Platform(edge, 'name', 'base', 'B', opt.B, 'Tc', opt.Tc, 'noreal3d');
            end
            % set arm
            plotopt = {'noname', 'nobase', 'tiles', 'floorlevel', 0, 'tilesize', 0.4, 'noshading', 'noshadow', 'nowrist'};
            if ~isempty(opt.qlim)
                mmarm = SerialLink(dh(4:end,:), 'name', 'arm', 'tool', Ht, 'plotopt', plotopt, 'qlim', opt.qlim);
            else
                mmarm = SerialLink(dh(4:end,:), 'name', 'arm', 'tool', Ht, 'plotopt', plotopt);
            end
            % construction
            Hb = mmplat.base;
            Ht = mmarm.tool;
            for i=1:size(dh,1)
                rod = Cuboid([dh(i,3),dh(i,3)/10,dh(i,3)/10]);
                if dh(i,end)
                    % The first 3 joints are from the platform,
                    % the joints of manipulator start from the 4th one.
                    if i>3
                        rodopt = {'m', rod.mass, 'r', [-rod.edge(1)/2,0,0], 'I', rod.inertia, 'B', opt.B(1,:), 'Tc', opt.Tc(1,:)};
                    else
                        rodopt = {'m', mmplat.body.mass, 'r', [0,0,0], 'I', mmplat.body.inertia, 'B', opt.B(1,:), 'Tc', opt.Tc(1,:)};
                    end
                    dhopt = {'theta', dh(i,1), 'a', dh(i,3), 'alpha', dh(i,4), 'prismatic'};
                else
                    if i>3
                        rodopt = {'m', rod.mass, 'r', [-rod.edge(1)/2,0,0], 'I', rod.inertia, 'B', opt.B(2,:), 'Tc', opt.Tc(2,:)};
                    else
                        rodopt = {'m', mmplat.body.mass, 'r', [0,0,0], 'I', mmplat.body.inertia, 'B', opt.B(2,:), 'Tc', opt.Tc(2,:)};
                    end
                    dhopt = {'d', dh(i,2), 'a', dh(i,3), 'alpha', dh(i,4), 'revolute'};
                end
                mmtree(i) = Link(dhopt{:},rodopt{:}); 
            end
            obj = obj@SerialLink(mmtree, 'name', opt.name, 'base', Hb, 'tool', Ht);
            obj.platform = mmplat;
            obj.arm = mmarm;
            obj.real3d = opt.real3d;
        end
        
        function h = plot(obj,varargin) 
            % Plot MobileRobot robot object
            
            % opt statement
            opt.workspace = [];
            opt.dim = 3;
            opt.frame = false;
            opt.framecolor = 'b';
            opt.framelength = sum(obj.platform.body.edge)/length(obj.platform.body.edge)/3;
            opt.framethick = 1;
            opt.framestyle = '-';
            opt.shading = false;
            opt.lightpos = [0 0 20];
            % opt parse: only stated fields are chosen to opt, otherwise to arg
            [opt,arg] = tb_optparse(opt, varargin); 
            % argument parse
            if length(arg)==1
                % get pose
                q = arg{1}(:)';
                if length(q)~=obj.n
                    error(['q should be 1x' num2str(obj.n)]);
                end
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
            % deal with some display options
            if opt.shading
                lighting gouraud
                light('position', opt.lightpos)
            end
            view(opt.dim); grid on;
            obj.animate(q,h.group);
        end 
        
        function animate(obj,q,handles)
            % Animate MobileRobot robot object
            
            if nargin < 3
                handles = findobj('Tag', obj.name);
            end
            % update pose
            qa = q(4:end); qb = q(1:3);
            obj.arm.base = obj.base*obj.links(1).A(qb(1))*obj.links(2).A(qb(2))*obj.links(3).A(qb(3));
            % animate
            for i=1:length(handles.Children) % draw frame first otherwise there will be delay
                if strcmp(get(handles.Children(i),'Tag'), [obj.name '-tool'])
                    set(handles.Children(i),'matrix',obj.arm.fkine(qa).T);
                end
                if strcmp(get(handles.Children(i),'Tag'), [obj.name '-' obj.platform.name])
                    obj.platform.animate(qb,handles.Children(i));
                end
                if strcmp(get(handles.Children(i),'Tag'), [obj.name '-' obj.arm.name])
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
            qa = q(4:end); qb = q(1:3);
            obj.arm.base = obj.base*obj.links(1).A(qb(1))*obj.links(2).A(qb(2))*obj.links(3).A(qb(3));
            
            group = hggroup('Tag', obj.name);
            h.group = group;
            h.arm = obj.arm.plot(qa,'workspace',opt.workspace);
            if opt.frame
                h.platform = obj.platform.plot(qb,'frame','framecolor', opt.framecolor,'framelength',opt.framelength, 'framethick', opt.framethick, 'framestyle', opt.framestyle);
                h.tool = SE3(obj.arm.fkine(qa)).plot('color', opt.framecolor,'length',opt.framelength, 'thick', opt.framethick, 'style', opt.framestyle);
                set(h.tool,'parent',group);
                set(h.tool,'Tag', [obj.name '-tool']);
            else
                h.platform = obj.platform.plot(qb);
            end
            set(h.arm.group,'Tag', [obj.name '-' obj.arm.name]);
            set(h.platform.group,'Tag', [obj.name '-' obj.platform.name]);
            set(h.arm.group,'parent',group);
            set(h.platform.group,'parent',group);
                
            % restore hold setting
            if ~ish
                hold off
            end
        end
    end
end