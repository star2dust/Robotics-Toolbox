% - Rigid Platform 3D Model class (rpy)
% (last mod.: 29-07-2020, Author: Chu Wu)
% Requires rvc & rte https://github.com/star2dust/Robotics-Toolbox
% Properties:
% - name: str
% - dynamics: mass(1x1), inertia(1x6), inerMat(6x6)
% - shapes: face(6x4), bvert(body)(8x3), edge(1x3)
% Methods:
% - Cuboid: construction (opt: name)
% - addDym: add dynamics
% - verts: get vertices in inertia frame
% - plot (opt: facecolor,facealpha, workspace, [no]frame, framecolor)
% - animate
% Methods (Static):
% - tobverts: get body vertices from edge
classdef Platform < Cuboid
    properties (SetAccess = protected) % all display variables are row vectors
        wheel = Cylinder
        mount = SE3
    end
    
    methods
        function obj = Platform(varargin)
            % P.Platform  Create Platform object
            
            % opt statement
            opt.name = 'plat';
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
            % platform body
            obj = obj@Cuboid(edge,'name',opt.name);
            obj.name = opt.name;
            % platform wheel
            for i=1:4
                obj.wheel(i) = Cylinder(edge(1)/6,edge(2)/6,'name',[opt.name '-wheel#' num2str(i)]);
                pwh = [(-1)^(i>2), (-1)^(mod(i,2)==0),1].*[edge(1)/2-edge(1)/6,edge(2)/2,-edge(3)/2];
                obj.mount(i) = SE3.qrpy([pwh,pi/2,0,0]);
            end
        end
        
        function obj = addDym(obj,mass)
            % C.addDym  Add dynamic parameters for Cuboid object
            ed = obj.edge;
            obj.mass = mass;
            obj.inertia = 1/12*mass*[ed(2)^2+ed(3)^2 ed(1)^2+ed(3)^2 ed(2)^2+ed(1)^2 0 0 0];
            obj.inerMat = [obj.mass*eye(3),zeros(3);zeros(3),diag(obj.inertia(1:3))+skew(obj.inertia(4:end))];
        end
        
        function h = plot(obj,varargin)  
            % C.plot  Plot Cuboid object
            
            % opt statement
            opt.facecolor = 'y';
            opt.facealpha = 0.8;
            opt.edgecolor = 'k';
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
            if opt.frame
                h = obj.plot@Cuboid(q,'workspace',opt.workspace,'facecolor',...
                    opt.facecolor, 'facealpha', opt.facealpha, 'edgecolor',...
                    opt.edgecolor, 'frame', 'framecolor', opt.framecolor,...
                    'framelength', opt.framelength, 'framethick',...
                    opt.framethick, 'framestyle', opt.framestyle);
            else
                h = obj.plot@Cuboid(q,'workspace',opt.workspace,'facecolor',...
                    opt.facecolor, 'facealpha', opt.facealpha, 'edgecolor',...
                    opt.edgecolor,'framecolor');
            end
            h = createWheel(obj,q,opt,h);
            view(3); grid on;
            obj.animate(q,h.group);
        end 
        
        function animate(obj,q,handles)
            % C.animate  Animate Cuboid object
            if nargin < 3
                handles = findobj('Tag', obj.name);
            end
            obj.animate@Cuboid(q,handles);
            for i=1:length(handles.Children)
                for j=1:length(obj.wheel)
                    if strcmp(get(handles.Children(i),'Tag'), [obj.name '-wheel#' num2str(j)])
                        qwh = toqrpy(SE3.qrpy(q)*obj.mount(j));
                        qwh(isnan(qwh))=0;
                        obj.wheel(j).animate(qwh,handles.Children(i));
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
        function [bvert,face] = tobvert(edge)
            % C.tobvert  Get vertices relative to body frame from edge
            templateVerts = [0,0,0;0,1,0;1,1,0;1,0,0;0,0,1;0,1,1;1,1,1;1,0,1];
            templateFaces = [1,2,3,4;5,6,7,8;1,2,6,5;3,4,8,7;1,4,8,5;2,3,7,6];
            % ^ y axis
            % | 6 % % 7 -> top
            % | % 2 3 % -> bottom
            % | % 1 4 % -> bottom
            % | 5 % % 8 -> top
            % -------> x axis
            face = templateFaces;
            bvert = [templateVerts(:,1)*edge(1)-edge(1)/2,...
                templateVerts(:,2)*edge(2)-edge(2)/2,...
                templateVerts(:,3)*edge(3)-edge(3)/2];
        end
    end
    
    methods (Access = protected)   
        function h = createWheel(obj,q,opt,h)
            % create an axis
            ish = ishold();
            if ~ishold
                % if hold is off, set the axis dimensions
                if ~isempty(opt.workspace)
                    axis(opt.workspace);
                end
                hold on
            end
            
            for i=1:length(obj.wheel)
                qwh = toqrpy(SE3.qrpy(q)*obj.mount(i));
                qwh(isnan(qwh))=0;
                h.wheel(i) = obj.wheel(i).plot(qwh, 'facecolor', opt.facecolor,...
                    'facealpha', opt.facealpha, 'edgecolor', opt.edgecolor);
                set(h.wheel(i).group,'parent',h.group);
            end
                      
            % restore hold setting
            if ~ish
                hold off
            end
        end
    end
end