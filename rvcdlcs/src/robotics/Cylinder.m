% - Rigid Cylinder 3D Model class (rpy)
% (last mod.: 30-07-2020, Author: Chu Wu)
% Requires rvc & rte https://github.com/star2dust/Robotics-Toolbox
% Properties:
% - name: str
% - bvert(body frame): 42x3
% - face: 20x4
% - radius: 1x1
% - height: 1x1
% - density: 1x1
% Methods:
% - Cuboid: construction
% - mass: get mass value
% - inertia: get inertia matrix
% - plot
% - animate
% - vert: get vertices in inertia frame
% Methods (Static):
% - tobvert: get body vertices from radius and height
classdef Cylinder < handle
    properties (SetAccess = protected) % all display variables are row vectors
        name
        % verts and edges
        bvert 
        face 
        radius 
        height
        % dynamic params
        density
    end
    
    methods
        function obj = Cylinder(varargin)
            % C.Cylinder  Create Cylinder object
            
            % opt statement
            opt.name = 'cyl';
            opt.density = 1;
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
                [obj.bvert,obj.face] = obj.tobvert(radius,height);
            else
                error('improper input dimension')
            end
            obj.density = opt.density;
        end
        
        function mas = mass(obj)
            % C.mass  Calculate mass value
            
            vol = pi*obj.radius^2*obj.height;
            mas = vol*obj.density;
        end
        
        function ine = inertia(obj)
            % C.inertia  Calculate inertia matrix
            
            % [Ixx Iyy Izz -Iyz Ixz -Ixy] vector relative to the body frame (6 dim)
            r = obj.radius; h = obj.height; m = obj.mass;
            vec = 1/12*m*[3*r^2+h^2 3*r^2+h^2 6*r^2 0 0 0];
            % how to calculate? => I = diag([Ixx Iyy Izz])+skew([-Iyz Ixz -Ixy])
            ine = diag(vec(1:3))+skew(vec(4:6));
        end
        
        function h = plot(obj,varargin)  
            % C.plot  Plot Cylinder object
            
            % opt statement
            opt.facecolor = 'y';
            opt.facealpha = 0.8;
            opt.edgecolor = 'k';
            opt.workspace = [];
            opt.frame = false;
            opt.framecolor = 'b';
            opt.framelength = obj.radius/3;
            opt.framethick = 1;
            opt.framestyle = '-';
            % opt parse: only stated fields are chosen to opt, otherwise to arg
            [opt,arg] = tb_optparse(opt, varargin); 
            if length(arg)==1
                q = arg{1}(:)';
            else
                error('unknown argument');
            end
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
            obj.animate(q,h.group);
        end 
        
        function animate(obj,q,handles)
            % C.animate  Animate Cylinder object
            if nargin < 3
                handles = findobj('Tag', obj.name);
            end
            for i=1:length(handles.Children)
                if strcmp(get(handles.Children(i),'Tag'), [obj.name '-frame'])
                    frame = SE3.qrpy(q);
                    set(handles.Children(i),'matrix',frame.T);
                elseif strcmp(get(handles.Children(i),'Tag'), [obj.name '-cylinder'])
                    set(handles.Children(i),'vertices',obj.vert(q),'faces',obj.face);
                else
                    vert = reshape(obj.vert(q)',6,21)';
                    if strcmp(get(handles.Children(i),'Tag'), [obj.name '-lower-surface'])
                        % plot lower surface
                        set(handles.Children(i),'vertices',vert(:,1:3), 'faces', 1:21);
                    else
                        % plot upper surface
                        set(handles.Children(i),'vertices',vert(:,4:6), 'faces', 1:21);
                    end
                end
            end
        end
        
        function vert = vert(obj,pose)
            % C.vert  Get vertices relative to inertia frame
            
            % frame update
            frame = SE3(pose(1:3))*SE3.rpy(pose(4:6));
            % verts position   
            vert = h2e(frame.T*e2h(obj.bvert'))';
        end
    end
    
    methods (Static)
        function [bvert,face] = tobvert(r,h)
            % C.tobvert  Get vertices relative to body frame from radius and height
            [X,Y,Z] = cylinder(r,20);
            [TRI,V]= surf2patch(X,Y,Z);
            bvert = [V(:,1:2),V(:,3)*h-h/2];
            face = TRI;
        end
    end
    
    methods (Access = private)   
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
            h.cyl = patch('vertices',obj.vert(q), 'faces', obj.face, 'facecolor', opt.facecolor, 'facealpha', opt.facealpha, 'edgecolor', opt.facecolor, 'parent', group);
            set(h.cyl,'Tag', [obj.name '-cylinder']);
            % plot lower surface
            vert = reshape(obj.vert(q)',6,21)';
            h.surl = patch('vertices',vert(:,1:3), 'faces', 1:21, 'facecolor', opt.facecolor, 'facealpha', opt.facealpha, 'edgecolor', opt.edgecolor, 'parent', group);
            set(h.surl,'Tag', [obj.name '-lower-surface']);
            % plot upper surface
            h.suru = patch('vertices',vert(:,4:6), 'faces', 1:21, 'facecolor', opt.facecolor, 'facealpha', opt.facealpha, 'edgecolor', opt.edgecolor, 'parent', group);
            set(h.suru,'Tag', [obj.name '-upper-surface']);
            
            if opt.frame
                frame = SE3.qrpy(q);
                h.frame = frame.plot('color', opt.framecolor, 'length', opt.framelength, 'thick', opt.framethick, 'style', opt.framestyle);
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