% Rigid Cuboid 3D Model class (rpy)
% (last mod.: 30-07-2020, Author: Chu Wu)
% Requires rvc & rte https://github.com/star2dust/Robotics-Toolbox
% Properties:
% - name: str
% - bvert(body frame): 8x3
% - face: 6x4
% - edge: 1x3 length(x) width(y) height(z)
% - density: 1x1
% Methods:
% - Cuboid: construction
% - mass: get mass value
% - inertia: get inertia matrix
% - plot
% - animate
% - vert: get vertices in inertia frame
% Methods (Static):
% - tobvert: get body vertices from edge
classdef Cuboid < handle
    properties (SetAccess = protected) % all display variables are row vectors
        name
        % verts and edges
        bvert 
        face 
        edge 
        % dynamic params
        density
    end
    
    methods
        function obj = Cuboid(varargin)
            % Create Cuboid object
            
            % opt statement
            opt.name = 'cub';
            opt.density = 1;
            % opt parse: only stated fields are chosen to opt, otherwise to arg
            [opt,arg] = tb_optparse(opt, varargin);       
            % argument parse
            if isempty(arg)
                edge = ones(1,3);
            elseif length(arg)==1
                edge = arg{1}(:)';
            else
                error('unknown arguments');
            end           
            % basic configuration
            if isvector(edge)&&length(edge)==3
                % weighted average (sum(weighList.*variableList,2)/sum(weighList))
                % parallel axis theorem (sum(weighList.*diag(variableList'*variableList)'))
                % verts list in body frame (format: [x y z])
                obj.edge = edge(:)';
                [obj.bvert,obj.face] = obj.tobvert(obj.edge);
            else
                error('improper input dimension')
            end
            % set options
            obj.name = opt.name;
            obj.density = opt.density;
        end
        
        function mas = mass(obj)
            % Calculate mass value
            
            vol = obj.edge(1)*obj.edge(2)*obj.edge(3);
            mas = vol*obj.density;
        end
        
        function ine = inertia(obj)
            % Calculate inertia matrix
            
            % [Ixx Iyy Izz Iyz Ixz Ixy] vector relative to the body frame (6 dim)
            e = obj.edge; m = obj.mass;
            vec = 1/12*m*[e(2)^2+e(3)^2,e(1)^2+e(3)^2,e(2)^2+e(1)^2,0,0,0];  
            % how to calculate? => I = diag([Ixx Iyy Izz])+skew([-Iyz Ixz -Ixy])
            ine = diag(vec(1:3))+skew([-1,1,-1].*vec(4:6));
        end
        
        function h = plot(obj,varargin)  
            % Plot Cuboid object
            
            % opt statement
            opt.facecolor = 'y';
            opt.facealpha = 0.8;
            opt.edgecolor = 'k';
            opt.edgealpha = 0.2;
            opt.workspace = [];
            opt.frame = false;
            opt.framecolor = 'b';
            opt.framelength = sum(obj.edge)/length(obj.edge)/3;
            opt.framethick = 1;
            opt.framestyle = '-';
            % opt parse: only stated fields are chosen to opt, otherwise to arg
            [opt,arg] = tb_optparse(opt, varargin); 
            if length(arg)==1
                q = arg{1}(:)';
            else
                error('unknown argument')
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
            obj.animate(q,h.group);
        end 
        
        function animate(obj,q,handles)
            % Animate Cuboid object
            
            if nargin < 3
                handles = findobj('Tag', obj.name);
            end
            for i=1:length(handles.Children)
                if strcmp(get(handles.Children(i),'Tag'), [obj.name '-cuboid'])
                    set(handles.Children(i),'vertices',obj.vert(q),'faces',obj.face);
                end
                if strcmp(get(handles.Children(i),'Tag'), [obj.name '-frame'])
                    frame = SE3.qrpy(q);
                    set(handles.Children(i),'matrix',frame.T);
                end
            end
        end
        
        function vert = vert(obj,pose)
            % Get vertices relative to inertia frame
            
            % frame update
            frame = SE3(pose(1:3))*SE3.rpy(pose(4:6));
            % verts position   
            vert = h2e(frame.T*e2h(obj.bvert'))';
        end
    end
    
    methods (Static)
        function [bvert,face] = tobvert(edge)
            % Get vertices relative to body frame from edge
            
            templateVerts = [0,0,0;0,1,0;1,1,0;1,0,0;0,0,1;0,1,1;1,1,1;1,0,1];
            templateFaces = [1,2,3,4;5,6,7,8;1,2,6,5;3,4,8,7;1,4,8,5;2,3,7,6];
            % ^ y axis
            % | 6 % % 7 -> top
            % | % 2 3 % -> bottom
            % | % 1 4 % -> bottom
            % | 5 % % 8 -> top
            % -------> x axis
            face = templateFaces;
            bvert = [templateVerts(:,1)*edge(1)-edge(1)/2,templateVerts(:,2)*edge(2)-edge(2)/2,templateVerts(:,3)*edge(3)-edge(3)/2];
        end
    end
    
    methods (Access = private)   
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
            
            if opt.frame
                frame = SE3.qrpy(q);
                h.frame = frame.plot('color', opt.framecolor,'length',...
                    opt.framelength, 'thick', opt.framethick, 'style', opt.framestyle);
                set(h.frame,'parent',group);
                set(h.frame,'Tag', [obj.name '-frame']);
            end
            
            h.cub = patch('vertices',obj.vert(q), 'faces', obj.face, 'facecolor',...
                opt.facecolor, 'facealpha', opt.facealpha, 'edgecolor',...
                opt.edgecolor, 'edgealpha', opt.edgealpha, 'parent', group);
            set(h.cub,'Tag', [obj.name '-cuboid']);
                    
            % restore hold setting
            if ~ish
                hold off
            end
        end
    end
end