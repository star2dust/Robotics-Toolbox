% Rigid Cylinder 3D Model class (rpy)
% (last mod.: 06-01-2020, Author: Chu Wu)
% Requires rvc & rte https://github.com/star2dust/Robotics-Toolbox
% Properties:
% - name: str
% - dynamics: mass(1x1), inertia(1x6), inerMat(6x6)
% - shapes: faces, bverts(body), radius(1x1), height(1x1)
% Methods:
% - Cylinder: construction (opt: name)
% - addDym: add dynamics
% - verts: get vertices in inertia frame
% - plot (opt: facecolor,facealpha, workspace, [no]frame, framecolor)
% - animate
classdef Cylinder < handle
    properties (SetAccess = protected) % all display variables are row vectors
        name
        % params
        mass % center of body frame (1 dim)
        inertia % [Ixx Iyy Izz -Iyz Ixz -Ixy] vector relative to the body frame (6 dim)
        % how to calculate? => I = diag([Ixx Iyy Izz])+skew([-Iyz Ixz -Ixy])
        inerMat % [M,0;0,I]
        % a list of verts and edges
        bverts % (body frame)
        faces %
        radius 
        height
    end
    
    methods
        function obj = Cylinder(varargin)
            % opt statement
            opt.name = 'cyl';
            % opt parse: only stated fields are chosen to opt, otherwise to arg
            [opt,arg] = tb_optparse(opt, varargin); 
            obj.name = opt.name;
            % argument parse
            if isempty(arg)
                radius = 1; height = 1;
            elseif length(arg)==2
                radius = arg{1}; height = arg{2};
            else
                error('unknown arguments');
            end           
            % basic configuration
            if isscalar(radius)&&isscalar(height)
                % verts list in body frame (format: [x y z])
                obj.radius = radius; obj.height = height;
                [obj.bverts,obj.faces] = obj.tobvert(radius,height);
            else
                error('improper input dimension')
            end
        end
        
        function obj = addDym(obj,m)
            r = obj.radius; h = obj.height; obj.mass = m;
            obj.inertia = 1/12*m*[3*r^2+h^2 3*r^2+h^2 6*r^2 0 0 0];
            obj.inerMat = [obj.mass*eye(3),zeros(3);zeros(3),diag(obj.inertia(1:3))+skew(obj.inertia(4:end))];
        end
        
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
            q = arg{2}(:)';
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
                        h = createCylinder(obj, q, opt);
                        % tag one of the graphical handles with the robot name and hang
                        % the handle structure off it
                        %                 set(handle.joint(1), 'Tag', robot.name);
                        %                 set(handle.joint(1), 'UserData', handle);
                    else
                        % create the robot 
                        newplot();
                        h = createCylinder(obj, q, opt);
                        set(gca, 'Tag', 'RTB.plot');
                    end 
                end      
            else
                % this axis never had a robot drawn in it before, let's use it
                h = createCylinder(obj, q, opt);
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
                if strcmp(get(handles.Children(i),'Tag'), [obj.name '-frame'])
                    frame = SE3.qrpy(q);
                    set(handles.Children(i),'matrix',frame.T);
                elseif strcmp(get(handles.Children(i),'Tag'), [obj.name '-cylinder'])
                    set(handles.Children(i),'vertices',obj.verts(q),'faces',obj.faces);
                else
                    verts = reshape(obj.verts(q)',6,21)';
                    if strcmp(get(handles.Children(i),'Tag'), [obj.name '-lower-surface'])
                        % plot lower surface
                        set(handles.Children(i),'vertices',verts(:,1:3), 'faces', 1:21);
                    else
                        % plot upper surface
                        set(handles.Children(i),'vertices',verts(:,4:6), 'faces', 1:21);
                    end
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
    
    methods (Static)
        function [bverts,faces] = tobvert(r,h)
            [X,Y,Z] = cylinder(r,20);
            [TRI,V]= surf2patch(X,Y,Z);
            bverts = [V(:,1:2),V(:,3)*h-h/2];
            faces = TRI;
        end
    end
    
    methods (Access = protected)   
        function h = createCylinder(obj,q,opt)
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
            % plot cylinder
            h.cyl = patch('vertices',obj.verts(q), 'faces', obj.faces, 'facecolor', opt.facecolor, 'facealpha', opt.facealpha, 'edgecolor', opt.facecolor, 'parent', group);
            set(h.cyl,'Tag', [obj.name '-cylinder']);
            % plot lower surface
            verts = reshape(obj.verts(q)',6,21)';
            h.surl = patch('vertices',verts(:,1:3), 'faces', 1:21, 'facecolor', opt.facecolor, 'facealpha', opt.facealpha, 'edgecolor', opt.edgecolor, 'parent', group);
            set(h.surl,'Tag', [obj.name '-lower-surface']);
            % plot upper surface
            h.suru = patch('vertices',verts(:,4:6), 'faces', 1:21, 'facecolor', opt.facecolor, 'facealpha', opt.facealpha, 'edgecolor', opt.edgecolor, 'parent', group);
            set(h.suru,'Tag', [obj.name '-upper-surface']);
            
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
    end
end