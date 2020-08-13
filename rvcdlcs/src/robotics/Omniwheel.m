% - Omni Wheel 3D Model class (rpy)
% (last mod.: 30-07-2020, Author: Chu Wu)
% Requires rvc & rte https://github.com/star2dust/Robotics-Toolbox
% Properties:
% - name: str
% - bvert(body frame): 42x3
% - face: 20x4
% - radius: 1x1
% - height: 1x1
% - density: 1x1
% - type: 45a or 45b
% Methods:
% - Omniwheel: construction
% - mass: get mass value
% - inertia: get inertia matrix
% - plot
% - animate
% - vert: get vertices in inertia frame
% Methods (Static):
% - tobvert: get body vertices from radius and height
classdef Omniwheel < Cylinder
    properties (SetAccess = protected) % all display variables are row vectors
        type
    end
    
    methods
        function obj = Omniwheel(varargin)
            % Create Omniwheel object
            
            % opt statement
            opt.name = 'whl';
            opt.density = 1;
            opt.type = '45a';
            % opt parse: only stated fields are chosen to opt, otherwise to arg
            [opt,arg] = tb_optparse(opt, varargin); 
            % argument parse
            if isempty(arg)
                radius = 1; height = 1;
            elseif length(arg)==2
                radius = arg{1}; height = arg{2};
            else
                error('unknown arguments');
            end
            obj = obj@Cylinder(radius,height,'name',opt.name,'density',opt.density);
            % reset body vertices
            if isscalar(radius)&&isscalar(height)
                % verts list in body frame (format: [x y z])
                obj.radius = radius; obj.height = height;
                [obj.bvert,obj.face] = obj.tobvert(radius,height,opt.type);
            else
                error('improper input dimension')
            end
            obj.type = opt.type;
        end
        
        function h = plot(obj,varargin)
            % Plot Cylinder object
            
            % opt statement
            opt.facecolor = 'y';
            opt.facealpha = 0.8;
            opt.edgecolor = 'k';
            opt.edgealpha = 0.1;
            opt.workspace = [];
            opt.frame = false;
            opt.framecolor = 'b';
            opt.framelength = obj.radius/3;
            opt.framethick = 1;
            opt.framestyle = '-';
            % opt parse: only stated fields are chosen to opt, otherwise to arg
            [opt,arg] = tb_optparse(opt, varargin); 
            if length(arg)==1
                q = arg{1};
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
                        h = createWheel(obj, q, opt);
                        % tag one of the graphical handles with the robot name and hang
                        % the handle structure off it
                        %                 set(handle.joint(1), 'Tag', robot.name);
                        %                 set(handle.joint(1), 'UserData', handle);
                    else
                        % create the robot 
                        newplot();
                        h = createWheel(obj, q, opt);
                        set(gca, 'Tag', 'RTB.plot');
                    end 
                end      
            else
                % this axis never had a robot drawn in it before, let's use it
                h = createWheel(obj, q, opt);
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
            % Animate Cylinder object
            
            if nargin < 3
                handles = findobj('Tag', obj.name);
            end
            for i=1:length(handles.Children)
                if strcmp(get(handles.Children(i),'Tag'), [obj.name '-frame'])
                    frame = SE3.qrpy(q);
                    set(handles.Children(i),'matrix',frame.T);
                else
                    set(handles.Children(i),'vertices',obj.vert(q),'faces',obj.face);
                end
            end
        end
    end
    
    methods (Static)
        function [bvert,face] = tobvert(r,h,t)
            % Get vertices relative to body frame from radius and height
            if nargin<3
                t='45a';
            end
            if t=="45a"||t=="45b"
                [F,V] = stlread(['omni' t '.stl']);
            else
               error('omni wheel type should be 45a or 45b') 
            end
            bvert = [V(:,1:2)/0.05*r,V(:,3)/0.04*h];
            face = F;
        end
    end
    
    methods (Access = private)   
        function h = createWheel(obj,q,opt)
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
            % plot wheel
            h.whl = patch('vertices',obj.vert(q), 'faces', obj.face, 'facecolor',...
                opt.facecolor, 'facealpha', opt.facealpha, 'edgecolor',...
                opt.edgecolor, 'edgealpha', opt.edgealpha, 'parent', group);
            set(h.whl,'Tag', [obj.name '-wheel']);
            
            if opt.frame
                frame = SE3.qrpy(q);
                h.frame = frame.plot('color', opt.framecolor, 'length',...
                    opt.framelength, 'thick', opt.framethick, 'style', opt.framestyle);
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