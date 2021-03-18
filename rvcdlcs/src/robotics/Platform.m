% - Onmi Platform 3D Model class (rpy)
% (last mod.: 03-08-2020, Author: Chu Wu)
% Requires rvc & rte https://github.com/star2dust/Robotics-Toolbox
% Properties:
% - wheel: Cylinder
% - body: Cuboid
% - Lwh: 1x1
% - lwh: 1x1
% - tree: Link
% - jacob: 4x3
% Methods:
% - Platform: construction
% - plot
% - animate
classdef Platform < SerialLink
    properties (SetAccess = protected) % all display variables are row vectors
        wheel
        body
        Lwh
        lwh
        tree
        jacob
        real3d
    end
    
    methods
        function obj = Platform(varargin)
            % Create Platform object
            
            % opt statement
            opt.B = zeros(2,1);
            opt.Tc = zeros(2,2);
            opt.dh = [0,0,0,-pi/2,1;
                -pi/2,0,0,pi/2,1;
                0,0,0,0,0];
            opt.r = 0.1;
            opt.h = 0.2;
            opt.real3d = false;
            opt.name = 'plat';
            % opt parse: only stated fields are chosen to opt, otherwise to arg
            [opt,arg] = tb_optparse(opt, varargin);
            % platform body
            cub = Youbot(arg{:},'name','body');
            % platform wheel
            namelist = ["fl" "fr" "bl" "br"];
            typelist = ["45a" "45b" "45b" "45a"];
            rwh = cub.edge(1)*opt.r;
            hwh = cub.edge(2)*opt.h;
            Lwh = cub.edge(1)/2-rwh;
            lwh = cub.edge(2)/2-7*hwh/16;
            for i=1:4
                wheel(i) = Omniwheel(rwh,hwh,'name',namelist(i),'type',typelist(i));
            end
            jacob = [1,-1,-(lwh+Lwh);
                1,1,(lwh+Lwh);
                1,1,-(lwh+Lwh);
                1,-1,(lwh+Lwh)]/rwh;
            % platform tree
            Hb = SE3([0,0,rwh+cub.edge(3)/2])*SE3(SO3.Ry(pi/2));
            Ht = SE3;
            trsopt = {'m', 0, 'r', [0,0,0], 'I', zeros(3),'B', opt.B(1,:), 'Tc', opt.Tc(1,:)};
            rotopt = {'m', cub.mass, 'r', [0,0,0], 'I', cub.inertia, 'B', opt.B(2,:), 'Tc', opt.Tc(2,:)};
            for i=1:3
                if opt.dh(i,end)
                    dhopt = {'theta', opt.dh(i,1), 'a', opt.dh(i,3), 'alpha', opt.dh(i,4), 'prismatic'};
                else
                    dhopt = {'d', opt.dh(i,2), 'a', opt.dh(i,3), 'alpha', opt.dh(i,4), 'revolute'};
                end
                if i==3
                    tree(i) = Link(dhopt{:},rotopt{:});
                else
                    tree(i) = Link(dhopt{:},trsopt{:});
                end
            end
            % construction
            obj = obj@SerialLink(tree, 'name', opt.name, 'base', Hb, 'tool', Ht);
            obj.body = cub;
            obj.wheel = wheel;
            obj.lwh = lwh;
            obj.Lwh = Lwh;
            obj.tree = tree;
            obj.jacob = jacob;
            obj.real3d = opt.real3d;
        end
        
        function h = plot(obj,varargin)
            % Plot Platform object
            
            % opt statement
            opt.facecolor = [.5 .5 .5];
            opt.facealpha = 0.9;
            opt.edgecolor = 'k';
            opt.edgealpha = 0;
            opt.workspace = [];
            opt.frame = false;
            opt.framecolor = 'b';
            opt.framelength = sum(obj.body.edge)/length(obj.body.edge)/3;
            opt.framethick = 1;
            opt.framestyle = '-';
            % opt parse: only stated fields are chosen to opt, otherwise to arg
            [opt,arg] = tb_optparse(opt, varargin);
            if length(arg)==1
                q = arg{1}(:)';
                if length(q)~=obj.n+length(obj.wheel)&&length(q)~=obj.n
                    error(['q should be 1x ' num2str(obj.n) ' or 1x' num2str(obj.n+length(obj.wheel))]);
                end
            else
                error('unknown argument')
            end
            h = createPlatform(obj,q,opt);
            view(3); grid on;
            obj.animate(q,h.group);
        end
        
        function animate(obj,q,handles)
            % Animate Platform object
            
            if nargin < 3
                handles = findobj('Tag', obj.name);
            end
            qb = toqrpy(obj.fkine(q(1:obj.n)));
            for i=1:length(handles.Children)
                if strcmp(get(handles.Children(i),'Tag'), [obj.name '-' obj.body.name])
                    if obj.real3d
                        obj.body.animate3d(qb,handles.Children(i));
                    else
                        obj.body.animate(qb,handles.Children(i));
                    end
                end
                for j=1:length(obj.wheel)
                    if strcmp(get(handles.Children(i),'Tag'), [obj.name '-' obj.wheel(j).name])
                        pwh = [(-1)^(j>2), (-1)^(mod(j,2)==0),1].*[obj.Lwh,obj.lwh,-obj.body.edge(3)/2];
                        if length(q)==obj.n
                            Twh = SE3.qrpy(qb)*SE3(pwh)*SE3(SO3.Rx(pi/2));
                        else
                            Twh = SE3.qrpy(qb)*SE3(pwh)*SE3(SO3.Rx(pi/2))*SE3(SO3.Rz(-q(obj.n+j)));
                        end
                        if obj.real3d
                            obj.wheel(j).animate3d(Twh,handles.Children(i));
                        else
                            obj.wheel(j).animate(Twh,handles.Children(i));
                        end
                    end
                end
            end
        end
    end
    
    methods (Access = private)
        function h = createPlatform(obj,q,opt)
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
            qb = toqrpy(obj.fkine(q(1:obj.n)));
            
            if obj.real3d
                if opt.frame
                    h.body = obj.body.plot3d(qb,'workspace',opt.workspace,'facecolor',...
                        opt.facecolor, 'facealpha', opt.facealpha, 'edgecolor',...
                        opt.edgecolor, 'edgealpha', opt.edgealpha, 'frame',...
                        'framecolor', opt.framecolor,...
                        'framelength', opt.framelength, 'framethick',...
                        opt.framethick, 'framestyle', opt.framestyle);
                else
                    h.body = obj.body.plot3d(qb,'workspace',opt.workspace,'facecolor',...
                        opt.facecolor, 'facealpha', opt.facealpha, 'edgecolor',...
                        opt.edgecolor,'framecolor','edgealpha',opt.edgealpha);
                end
            else
                if opt.frame
                    h.body = obj.body.plot(qb,'workspace',opt.workspace,'facecolor',...
                        opt.facecolor, 'facealpha', opt.facealpha, 'edgecolor',...
                        opt.edgecolor, 'edgealpha', opt.edgealpha, 'frame',...
                        'framecolor', opt.framecolor,...
                        'framelength', opt.framelength, 'framethick',...
                        opt.framethick, 'framestyle', opt.framestyle);
                else
                    h.body = obj.body.plot(qb,'workspace',opt.workspace,'facecolor',...
                        opt.facecolor, 'facealpha', opt.facealpha, 'edgecolor',...
                        opt.edgecolor,'framecolor','edgealpha',opt.edgealpha);
                end
            end
            set(h.body.group, 'parent', group);
            set(h.body.group, 'Tag', [obj.name '-' obj.body.name]);
            
            wheelcolor = [0.9290, 0.6940, 0.1250];
            for i=1:length(obj.wheel)
                pwh = [(-1)^(i>2), (-1)^(mod(i,2)==0),1].*[obj.Lwh,obj.lwh,-obj.body.edge(3)/2];
                if length(q)==obj.n
                    Twh = SE3.qrpy(qb)*SE3(pwh)*SE3(SO3.Rx(pi/2));
                else
                    Twh = SE3.qrpy(qb)*SE3(pwh)*SE3(SO3.Rx(pi/2))*SE3(SO3.Rz(-q(obj.n+i)));
                end
                if obj.real3d
                    if opt.frame&&length(q)==obj.n+length(obj.wheel)
                        h.wheel(i) = obj.wheel(i).plot3d(Twh, 'facecolor', wheelcolor,...
                            'facealpha', opt.facealpha, 'edgecolor', opt.edgecolor,...
                            'edgealpha',opt.edgealpha, 'frame',...
                            'framecolor', opt.framecolor,...
                            'framelength', opt.framelength, 'framethick',...
                            opt.framethick, 'framestyle', opt.framestyle);
                    else
                        h.wheel(i) = obj.wheel(i).plot3d(Twh, 'facecolor', wheelcolor,...
                            'facealpha', opt.facealpha, 'edgecolor', opt.edgecolor,...
                            'edgealpha',opt.edgealpha);
                    end
                else
                    if opt.frame&&length(q)==obj.n+length(obj.wheel)
                        h.wheel(i) = obj.wheel(i).plot(Twh, 'facecolor', wheelcolor,...
                            'facealpha', opt.facealpha, 'edgecolor', opt.edgecolor,...
                            'edgealpha',opt.edgealpha, 'frame',...
                            'framecolor', opt.framecolor,...
                            'framelength', opt.framelength, 'framethick',...
                            opt.framethick, 'framestyle', opt.framestyle);
                    else
                        h.wheel(i) = obj.wheel(i).plot(Twh, 'facecolor', wheelcolor,...
                            'facealpha', opt.facealpha, 'edgecolor', opt.edgecolor,...
                            'edgealpha',opt.edgealpha);
                    end
                end
                set(h.wheel(i).group, 'parent', group);
                set(h.wheel(i).group,'Tag',[obj.name '-' obj.wheel(i).name]);
            end
            
            % restore hold setting
            if ~ish
                hold off
            end
        end
    end
end