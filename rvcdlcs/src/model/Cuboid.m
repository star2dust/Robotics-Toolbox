% Rigid Cuboid 3D Model class (rpy)
% (last mod.: 01-01-2020, Author: Chu Wu)
% Requires rvc & rte https://github.com/star2dust/Robotics-Toolbox
% Properties:
% - name: str
% - dynamics parameters: mass(1x1), inertia(1x6), inerMat(6x6)
% - shapes: faces(6x4), edges(1x3), bverts(body)(8x3)
% Methods:
% - Cuboid: construction (opt: name)
% - verts: get vertices in inertia frame
% - plot (opt: facecolor,facealpha, workspace, [no]frame, framecolor)
% - animate
classdef Cuboid < handle
    properties (SetAccess = protected) % all display variables are row vectors
        name
        % translation & rotation states in world frame (6 dim)
        % angle expressed in Euler angle rotations "ZYX" 
        % pose % [x,y,z,thr,thp,thy]
        % frames (SE3) 
        % frame % (R = rotz(thy)*roty(thp)*rotx(thr)=SE3.rpy([thr,thp,thy]))
        % params
        mass % center of body frame (1 dim)
        inertia % [Ixx Iyy Izz -Iyz Ixz -Ixy] vector relative to the body frame (6 dim)
        % how to calculate? => I = diag([Ixx Iyy Izz])+skew([-Iyz Ixz -Ixy])
        inerMat % [M,0;0,I]
        % a list of verts and edges
        bverts % (8x3) (body frame)
        % verts % (8x3) (inertia frame)
        faces % (6x4)
        edges % (1x3) depth(x) width(y) height(z)
    end
    
    properties (Constant, Access = private)
        templateVerts = [0,0,0;0,1,0;1,1,0;1,0,0;0,0,1;0,1,1;1,1,1;1,0,1];
        templateFaces = [1,2,3,4;5,6,7,8;1,2,6,5;3,4,8,7;1,4,8,5;2,3,7,6];
        % ^ y axis
        % | 6 % % 7 -> top
        % | % 2 3 % -> bottom
        % | % 1 4 % -> bottom
        % | 5 % % 8 -> top
        % -------> x axis
    end
    
    methods
        function obj = Cuboid(varargin)
            % opt statement
            opt.name = [];
            % opt parse: only stated fields are chosen to opt, otherwise to arg
            [opt,arg] = tb_optparse(opt, varargin); 
            obj.name = opt.name;
            % argument parse
            if isempty(arg)
                edges = ones(1,3);
            elseif length(arg)==1
                edges = arg{1};
            else
                error('unknown arguments');
            end           
            % basic configuration
            if isvector(edges)&&length(edges)==3
                % weighted average (sum(weighList.*variableList,2)/sum(weighList))
                % parallel axis theorem (sum(weighList.*diag(variableList'*variableList)'))
                % verts list in body frame (format: [x y z])
                obj.edges = edges(:)';
                obj.edge2body;
                % update pose
%                 obj.update(zeros(1,6));
            else
                error('Improper input dimension')
            end
        end
        
        function obj = addDynParam(obj,mass)
            edge = obj.edges;
            obj.mass = mass;
            obj.inertia = 1/12*mass*[edge(2)^2+edge(3)^2 edge(1)^2+edge(3)^2 edge(2)^2+edge(1)^2 0 0 0];
            obj.inerMat = [obj.mass*eye(3),zeros(3);zeros(3),diag(obj.inertia(1:3))+skew(obj.inertia(4:end))];
        end
        
%         function obj = update(obj,pose)
%             obj.pose = pose(:)';
%             % regulate angles within [-pi,pi]
%             obj.pose(4:6) = mod(obj.pose(4:6)+pi,2*pi)-pi;
%             obj.frame = SE3.qrpy(obj.pose);
%             % update verts
%             obj.updateVerts;
%         end
        
        function h = plot(varargin)  
            % opt statement
            opt.facecolor = 'y';
            opt.facealpha = 0.8;
            opt.edgecolor = 'k';
            opt.workspace = [];
            opt.frame = false;
            opt.framecolor = 'b';
            % opt parse: only stated fields are chosen to opt, otherwise to arg
            [opt,arg] = tb_optparse(opt, varargin); 
            obj = arg{1};
            q = arg{2};
            % update pose
            % obj.update(q);
            % logic to handle where the plot is drawn, are old figures updated or
            % replaced?
            %  calls create_floor() and create_robot() as required.
            if strcmp(get(gca,'Tag'), 'RTB.plot')
                % this axis is an RTB plot window          
                rhandles = findobj('Tag', obj.name);            
                if isempty(rhandles)
                    % this robot doesnt exist here, create it or add it  
                    if ishold
                        % hold is on, add the robot, don't change the floor
                        h = createCuboid(obj, q, opt);
                        % tag one of the graphical handles with the robot name and hang
                        % the handle structure off it
                        %                 set(handle.joint(1), 'Tag', robot.name);
                        %                 set(handle.joint(1), 'UserData', handle);
                    else
                        % create the robot 
                        newplot();
                        h = createCuboid(obj, q, opt);
                        set(gca, 'Tag', 'RTB.plot');
                    end 
                end      
            else
                % this axis never had a robot drawn in it before, let's use it
                h = createCuboid(obj, q, opt);
                set(gca, 'Tag', 'RTB.plot');
                set(gcf, 'Units', 'Normalized');
                pf = get(gcf, 'Position');
                %         if strcmp( get(gcf, 'WindowStyle'), 'docked') == 0
                %             set(gcf, 'Position', [0.1 1-pf(4) pf(3) pf(4)]);
                %         end
            end
            view(3); grid on;
            obj.animate(q);
        end 
        
        function animate(obj,q)
            if nargin < 3
                handles = findobj('Tag', obj.name);
            end
            for i=1:length(handles.Children)
                if strcmp(get(handles.Children(i),'Tag'), [obj.name '-cuboid'])
                    set(handles.Children(i),'vertices',obj.verts(q),'faces',obj.faces);
                else
                    frame = SE3.qrpy(q);
                    set(handles.Children(i),'matrix',frame.T);
                end
            end
        end
        
        function verts = verts(obj,pose)
            % frame update
            frame = SE3(pose(1:3))*SE3.rpy(pose(4:6));
            % verts position   
            verts = h2e(frame.T*e2h(obj.bverts'))';
        end
    end
    
    methods (Access = protected)   
        function h = createCuboid(obj,q,opt)
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
            h.cub = patch('vertices',obj.verts(q), 'faces', obj.faces, 'facecolor', opt.facecolor, 'facealpha', opt.facealpha, 'edgecolor', opt.edgecolor, 'parent', group);
            set(h.cub,'Tag', [obj.name '-cuboid']);
            
            if opt.frame
                frame = SE3.qrpy(q);
                h.frame = frame.plot('color', opt.framecolor);
                set(h.frame,'parent',group);
                set(h.frame,'Tag', [obj.name '-frame']);
            end
                      
            % restore hold setting
            if ~ish
                hold off
            end
        end
        
        function obj = edge2body(obj)
           obj.faces = obj.templateFaces;
           obj.bverts = [obj.templateVerts(:,1)*obj.edges(1)-obj.edges(1)/2,obj.templateVerts(:,2)*obj.edges(2)-obj.edges(2)/2,obj.templateVerts(:,3)*obj.edges(3)-obj.edges(3)/2]; 
        end
    end
end