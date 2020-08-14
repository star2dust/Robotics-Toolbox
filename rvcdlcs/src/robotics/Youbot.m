% - Youbot Platform 3D Model class (rpy)
% (last mod.: 30-07-2020, Author: Chu Wu)
% Requires rvc & rte https://github.com/star2dust/Robotics-Toolbox
% Properties:
% - name: str
% - bvert3d(body frame) 
% - face3d 
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
classdef Youbot < Cuboid
    properties (SetAccess = protected) % all display variables are row vectors
        bvert3d
        face3d
    end
    
    methods
        function obj = Youbot(varargin)
            % Create Youbot object
            
            % opt statement
            opt.name = 'bot';
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
            obj = obj@Cuboid(edge,'name',opt.name,'density',opt.density);
            % basic configuration
            if isvector(edge)&&length(edge)==3
                % weighted average (sum(weighList.*variableList,2)/sum(weighList))
                % parallel axis theorem (sum(weighList.*diag(variableList'*variableList)'))
                % verts list in body frame (format: [x y z])
                obj.edge = edge(:)';
                [obj.bvert3d,obj.face3d] = obj.tobvert3d(obj.edge);
            else
                error('improper input dimension')
            end
            % set options
            obj.name = opt.name;
            obj.density = opt.density;
        end
        
        function h = plot3d(obj,varargin)  
            % Plot Youbot object
            
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
                        h = createYoubot(obj, q, opt);
                        % tag one of the graphical handles with the robot name and hang
                        % the handle structure off it
                        %                 set(handle.joint(1), 'Tag', robot.name);
                        %                 set(handle.joint(1), 'UserData', handle);
                    else
                        % create the robot 
                        newplot();
                        h = createYoubot(obj, q, opt);
                        set(gca, 'Tag', 'RTB.plot');
                    end 
                end      
            else
                % this axis never had a robot drawn in it before, let's use it
                h = createYoubot(obj, q, opt);
                set(gca, 'Tag', 'RTB.plot');
                set(gcf, 'Units', 'Normalized');
                pf = get(gcf, 'Position');
                %         if strcmp( get(gcf, 'WindowStyle'), 'docked') == 0
                %             set(gcf, 'Position', [0.1 1-pf(4) pf(3) pf(4)]);
                %         end
            end
            view(3); grid on;
            obj.animate3d(q,h.group);
        end 
        
        function animate3d(obj,q,handles)
            % Animate Youbot object
            
            if nargin < 3
                handles = findobj('Tag', obj.name);
            end
            for i=1:length(handles.Children)
                for j=1:length(obj.bvert3d)
                    if strcmp(get(handles.Children(i),'Tag'), [obj.name '-youbot' num2str(j)])
                        vert3d = obj.vert3d(q);
                        set(handles.Children(i),'vertices',vert3d{j}, 'faces', obj.face3d{j});
                    end
                end
                if strcmp(get(handles.Children(i),'Tag'), [obj.name '-frame'])
                    frame = SE3.qrpy(q);
                    set(handles.Children(i),'matrix',frame.T);
                end
            end
        end
        
        function vert3d = vert3d(obj,pose)
            % Get vertices relative to inertia frame
            
            % frame update
            frame = SE3(pose(1:3))*SE3.rpy(pose(4:6));
            % verts position   
            for i=1:length(obj.bvert3d)
                vert3d{i} = h2e(frame.T*e2h(obj.bvert3d{i}'))';
            end
        end
    end
    
    methods (Static)
        function [bvert3d,face3d] = tobvert3d(edge)
            % Get vertices relative to body frame from edge
            
            for i=1:2
                [F{i},V{i}] = stlread(['youbot' num2str(i) '.stl']);
                bvert3d{i} = [V{i}(:,1)/0.55*edge(1),V{i}(:,2)/0.35*edge(2),...
                    V{i}(:,3)/0.1*edge(3)];
                face3d{i} = F{i};
            end
        end
    end
    
    methods (Access = private)   
        function h = createYoubot(obj,q,opt)
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
            facecolor = {opt.facecolor, [0.8,0.8,0.8]};
            edgecolor = facecolor;
            for i=1:length(obj.bvert3d)
                vert3d = obj.vert3d(q);
                h.bot(i) = patch('vertices',vert3d{i}, 'faces', obj.face3d{i}, 'facecolor',...
                    facecolor{i}, 'facealpha', opt.facealpha, 'edgecolor',...
                    edgecolor{i}, 'edgealpha', opt.edgealpha, 'parent', group);
                set(h.bot(i),'Tag', [obj.name '-youbot' num2str(i)]);
            end
            
            if opt.frame
                frame = SE3.qrpy(q);
                h.frame = frame.plot('color', opt.framecolor,'length',...
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