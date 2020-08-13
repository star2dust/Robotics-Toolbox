% - Onmi Platform 3D Model class (rpy)
% (last mod.: 03-08-2020, Author: Chu Wu)
% Requires rvc & rte https://github.com/star2dust/Robotics-Toolbox
% Properties:
% - wheel: Cylinder
% - body: Cuboid
% - mount: SE3
% - tree: Link
% Methods:
% - Platform: construction 
% - plot
% - animate
classdef Platform < SerialLink
    properties (SetAccess = protected) % all display variables are row vectors
        wheel
        body
        mount
        tree
    end
    
    methods
        function obj = Platform(varargin)
            % P.Platform  Create Platform object
            
            % opt statement
            opt.B = zeros(2,1);
            opt.Tc = zeros(2,2);
            opt.dh = [0,0,0,-pi/2,1;
                -pi/2,0,0,pi/2,1;
                0,0,0,0,0];
            opt.name = 'plat';
            % opt parse: only stated fields are chosen to opt, otherwise to arg
            [opt,arg] = tb_optparse(opt, varargin);  
            % platform body
            cub = Cuboid(arg{:},'name','body');
            % platform wheel
            namelist = ["fl" "fr" "bl" "br"];
            typelist = ["45a" "45b" "45b" "45a"];
            for i=1:4
                wheel(i) = Omniwheel(cub.edge(1)/6,cub.edge(2)/6,'name',namelist(i),'type',typelist(i));
                pwh = [(-1)^(i>2), (-1)^(mod(i,2)==0),1].*[cub.edge(1)/2-cub.edge(1)/6,cub.edge(2)/2,-cub.edge(3)/2];
                mount(i) = SE3.qrpy([pwh,pi/2,0,0]);
            end
            % platform tree
            Hb = SE3([0,0,cub.edge(1)/6+cub.edge(3)/2])*SE3(SO3.Ry(pi/2));
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
            obj.mount = mount;
            obj.tree = tree;
        end
        
        function h = plot(obj,varargin)  
            % C.plot  Plot Cuboid object
            
            % opt statement
            opt.facecolor = 'y';
            opt.facealpha = 0.8;
            opt.edgecolor = [0.1 0.1 0.1];
            opt.edgealpha = 0.5;
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
                if length(q)~=obj.n&&length(q)~=length(obj.wheel)
                   error(['q should be 1x' num2str(obj.n) 'or 1x' num2str(length(obj.wheel))]); 
                end
            else
                error('unknown argument')
            end
            h = createPlatform(obj,q,opt);
            view(3); grid on;
            obj.animate(q,h.group);
        end 
        
        function animate(obj,q,handles)
            % C.animate  Animate Cuboid object
            if nargin < 3
                handles = findobj('Tag', obj.name);
            end
            qb = toqrpy(obj.fkine(q));
            for i=1:length(handles.Children)
                if strcmp(get(handles.Children(i),'Tag'), [obj.name '-' obj.body.name])
                    obj.body.animate(qb,handles.Children(i));
                end
                for j=1:length(obj.wheel)
                    if strcmp(get(handles.Children(i),'Tag'), [obj.name '-' obj.wheel(j).name])
                        qwh = toqrpy(SE3.qrpy(qb)*obj.mount(j));
                        qwh(isnan(qwh))=0;
                        obj.wheel(j).animate(qwh,handles.Children(i));
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
            qb = toqrpy(obj.fkine(q));
            
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
            set(h.body.group, 'parent', group);
            set(h.body.group, 'Tag', [obj.name '-' obj.body.name]);
            
            for i=1:length(obj.wheel)
                qwh = toqrpy(SE3.qrpy(qb)*obj.mount(i));
                qwh(isnan(qwh))=0;
                h.wheel(i) = obj.wheel(i).plot(qwh, 'facecolor', opt.facecolor,...
                    'facealpha', opt.facealpha, 'edgecolor', opt.edgecolor,...
                    'edgealpha',opt.edgealpha*0.2);
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