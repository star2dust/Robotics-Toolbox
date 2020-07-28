% - Map 2D/3D Model class (SE3, rpy, stdDH)
% (last mod.: 02-04-2020, Author: Chu Wu)
% Requires rvc & rte https://github.com/star2dust/Robotics-Toolbox
% Properties:
% - name: str (map*)
% - map: map matrix (1: obstacles , 0: otherwise)
% - tile: size of tiles (1x1)
% - Af: Adjacency matrix of obstacle-free graph (length(map(:))xlength(map(:)))
% - Ao: Adjacency matrix of obstacle graph (length(map(:))xlength(map(:)))
% - Voc: Vertices of obstacle in each connected component (nx2 or nx3)
% Methods:
% - Map: construction (arg: map) 
% (opt: name, tile)
% - plot (no arg)
% (opt: workspace, dim, obheight, obcolor, obthick, bastyle, bacolor, bathick)
% Methods (Static):
classdef Map < handle
    properties
        name
        matrix
        siz
        tile
        Af
        Ao
        indoc
        Voc
        route
    end
    
    methods
        function obj = Map(varargin)
            % M.Map  Create Map object
            
            % opt statement
            opt.name = 'map';
            opt.tile = 1;
            opt.graph = true;
            opt.planning = true;
            opt.start = 1;
            % opt parse: only stated fields are chosen to opt, otherwise to arg
            [opt,arg] = tb_optparse(opt, varargin);
            % check validity
            if length(arg)==1
                matrix = arg{1};
            else
                error('unknown argument')
            end
            % read from picture or matrix
            if ischar(matrix)
                obj.matrix = readMap(matrix);
            else
                obj.matrix = matrix;
            end
            % struct
            obj.name = opt.name;
            obj.siz = size(obj.matrix);
            obj.tile = opt.tile;
            if opt.graph
                obj.Ao = Map.map2gphA(obj.matrix,1);
                obj.indoc = Map.map2indoc(obj.matrix);
                obj.Voc = Map.indoc2Voc(obj.siz,obj.tile,obj.indoc);
            end
            if opt.planning
                obj.Af = Map.map2gphA(obj.matrix,0);  
                n = length(obj.matrix(:));
                sub = ind2sub_(obj.siz,1:n);
                obj.route = dijkstra(obj.Af,sub,opt.start);
            end
        end
        
        function h = plot(obj,varargin)
            % M.plot  Plot Map object
            
            % opt statement
            opt.dim = length(size(obj.matrix));
            if opt.dim==2
                opt.workspace = [0 obj.tile*size(obj.matrix,1) 0 obj.tile*size(obj.matrix,2)];
            else
                opt.workspace = [0 obj.tile*size(obj.matrix,1) 0 obj.tile*size(obj.matrix,2) 0 obj.tile*size(obj.matrix,3)];
            end
            opt.shrink = 0;
            opt.route = false;
            opt.goal = length(obj.matrix(:));
            opt.obheight = 0.5;
            opt.obcolor = 'k';
            opt.obthick = 1;
            opt.bastyle = '-';
            opt.bacolor = 'k';
            opt.bathick = 1;
            % opt parse: only stated fields are chosen to opt, otherwise to arg
            [opt,arg] = tb_optparse(opt, varargin);
            % argument parse
            if ~isempty(arg)
                error('unknown arguments');
            end
            if strcmp(get(gca,'Tag'), 'RTB.plot')
                % this axis is an RTB plot window
                rhandles = findobj('Tag', obj.name);
                if isempty(rhandles)
                    % this robot doesnt exist here, create it or add it
                    if ishold
                        % hold is on, add the robot, don't change the floor
                        h = createMap(obj, opt);
                        % tag one of the graphical handles with the robot name and hang
                        % the handle structure off it
                        %                 set(handle.joint(1), 'Tag', robot.name);
                        %                 set(handle.joint(1), 'UserData', handle);
                    else
                        % create the robot
                        newplot();
                        h = createMap(obj, opt);
                        set(gca, 'Tag', 'RTB.plot');
                    end
                end
            else
                % this axis never had a robot drawn in it before, let's use it
                h = createMap(obj, opt);
                set(gca, 'Tag', 'RTB.plot');
                set(gcf, 'Units', 'Normalized');
                %         pf = get(gcf, 'Position');
                %         if strcmp( get(gcf, 'WindowStyle'), 'docked') == 0
                %             set(gcf, 'Position', [0.1 1-pf(4) pf(3) pf(4)]);
                %         end
            end
            view(opt.dim); grid on; 
            if opt.dim==3
                rotate3d on
            end
        end
        
        function route = astar(obj,start,goal)
            n = length(obj.matrix(:));
            sub = ind2sub_(obj.siz,1:n);
            route = astar(obj.Af,sub,start,goal);
        end
    end
    
    methods (Access = protected)
        function h = createMap(obj, opt)
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
            
            % obstacles
            for i=1:length(obj.Voc)
                I = boundary(obj.Voc{i},opt.shrink);
%                 I = 1:size(obj.Voc{i},1);
                if opt.dim == 2      
                    h.obstacle(i) = patch('vertices', obj.Voc{i}(I,:), 'faces', 1:length(I),...
                        'facecolor', opt.obcolor, 'facealpha', opt.obthick,...
                        'edgecolor', opt.obcolor, 'edgealpha', opt.obthick, 'parent', group);
                else
                    X = kron(ones(2,1),obj.Voc{i}(I,1)');
                    Y = kron(ones(2,1),obj.Voc{i}(I,2)');
                    if length(obj.siz)==2
                        m = length(obj.Voc{i}(I,1));
                        Z = [zeros(1,m);ones(1,m)]*opt.obheight;
                        [Fo,Vo]= surf2patch(X,Y,Z);
                        h.obstacle(i) = patch('vertices', Vo, 'faces', Fo,...
                            'facecolor', opt.obcolor, 'facealpha', opt.obthick,...
                            'edgecolor', opt.obcolor, 'edgealpha', opt.obthick, 'parent', group);
                        Xr = X'; Yr = Y'; Zr = Z';
                        Vr = [Xr(:),Yr(:),Zr(:)]; Fr = [1:size(X,2);size(X,2)+1:size(X,2)*2];
                        h.roof(i) = patch('vertices', Vr, 'faces', Fr,...
                            'facecolor', opt.obcolor, 'facealpha', opt.obthick,...
                            'edgecolor', opt.obcolor, 'edgealpha', opt.obthick, 'parent', group);
                        set(h.roof(i),'Tag', [obj.name '-roof' num2str(i)]);
                    else
                        % trisurf(k2,x,y,z,'FaceColor','cyan')
                        error('features to be added');
                    end
                end
                set(h.obstacle(i),'Tag', [obj.name '-obstacle' num2str(i)]);
            end
%             % barrier
%             subb = [1,1;obj.siz(1),1;obj.siz;1,obj.siz(2)];
%             subbsplit = num2cell(subb,1);
%             indb = sub2ind(obj.siz,subbsplit{:});
%             locb = Map.ind2loc(obj.siz,obj.tile,indb);
%             Vb = Map.loc2vert(obj.siz,obj.tile,locb);
%             Ib = convhull_(Vb);
%             h.barrier = line(Vb(Ib,1),Vb(Ib,2),'Color',opt.bacolor,'LineStyle', opt.bastyle, 'LineWidth', opt.bathick, 'parent', group);
%             set(h.barrier,'Tag', [obj.name '-barrier']);
            
            % route
            if opt.route
                locru = Map.ind2loc(obj.siz,obj.tile,obj.route{opt.goal});
                h.route = line(locru(:,1),locru(:,2),'Color',opt.bacolor,'LineStyle', opt.bastyle, 'LineWidth', opt.bathick, 'parent', group);
                set(h.route,'Tag', [obj.name '-route']);
            end
            
            % restore hold setting
            if ~ish
                hold off
            end
        end
    end
    
    methods (Static)
        function matrix = randMap(range,pct)
            % M.randMap  Randomly generate map matrix by percentage
            % - range: size of map matrix
            % - pct: percentage of obstacles (from 0 to 1) (0: no obs, 1: full obs)
            % - matrix: map matrix with elements 0 or 1
            matrix = rand(range)<pct;
        end
        
        function matrix = readMap(str)
            % M.readMap  Read map matrix from a picture
            % - str: path of the picture file
            % - matrix: map matrix with elements 0 or 1
            f1=imread(str);
            bw1=imbinarize(f1);%ʹ��Ĭ��ֵ0.5
            matrix = ~bw1(:,:,1);
        end
        
        function matrix = dilateMap(matrix,str,pixel)
            % M.dilateMap  Dilate map matrix by pixel size
            % - matrix: map matrix with elements 0 or 1
            % - str, siz: see strel function
            % (eg. 2D: 'octagon',r; 3D: 'sphere',r)
            SE = strel(str,ceil(pixel));
            matrix = imdilate(matrix,SE);
        end
        
        function varargout = map2gphA(matrix,type)
            % M.map2gphA  Get adjacency matrix of obstacle-free(type 0) or 
            % obstacle(type 1) graph from map matrix
            % - matrix: map matrix with elements 0 or 1
            % - A: Adjacency matrix of graph
            nx = size(matrix,1); ny = size(matrix,2); nz = size(matrix,3); n = nx*ny*nz;
            Ab = zeros(n); Af = zeros(n);
            for i=1:n-1
                for j=i+1:n
                    [ix,iy,iz] = ind2sub(size(matrix),i);
                    [jx,jy,jz] = ind2sub(size(matrix),j);
                    if nargin<2||type==0
                        if matrix(i)==0&&matrix(j)==0
                            % any vertice pair inside an obstacle free district has an edge
                            ijx = min(ix,jx):max(ix,jx);
                            ijy = min(iy,jy):max(iy,jy);
                            ijz = min(iz,jz):max(iz,jz);
                            if sum(sum(matrix(ijx,ijy,ijz)))==0
                                Af(i,j) = 1;
                                Af(j,i) = 1;
                            end
                        end
                    end
                    if nargin<2||type==1
                        if matrix(i)==1&&matrix(j)==1
                            % any adjancent vertice pair inside an obstacle district has an edge
                            ijx = max(ix,jx)-min(ix,jx);
                            ijy = max(iy,jy)-min(iy,jy);
                            ijz = max(iz,jz)-min(iz,jz);
                            if ijx<=1&&ijy<=1&&ijz<=1
                                Ab(i,j) = 1;
                                Ab(j,i) = 1;
                            end
                        end
                    end
                    if nargin<2
                        varargout = {Af,Ab};
                    else
                        switch type
                            case 0
                                varargout = {Af};
                            case 1
                                varargout = {Ab};
                            otherwise
                                error('unknown type')
                        end
                    end
                end
            end
        end
        
        function indoc = map2indoc(matrix)
            % M.map2indoc  Get the connected components of obstacle graph
            % from map matrix, where nc = lenth(indoc) is the number of all 
            % connected components and indoc{i} contains all index of tiles
            % lying in the ith connected component.
            % - matrix: map matrix with elements 0 or 1
            % - indoc: tile indices in each connected component (cell 1xnc)
            
            % get Ao from matrix
            import Map.*
            Ao = map2gphA(matrix,1);
            indo = 1:size(Ao,1);
            % discard obstacle-free nodes
            Ao1 = Ao(matrix==1,matrix==1);
            indo1 = indo(matrix==1);
            % generate graph G (nxn)
            Go1 = graph(Ao1);
            % find connected component of G
            % - Co: component id of each node (1xn)(m=max(C))
            % - noc: number of obstacle nodes in each component (1xm)
            [Co1,noc1] = conncomp(Go1);
            indoc = cell(1,length(noc1));
            for i=1:length(noc1)
                indoc{i} = indo1(Co1==i);
            end
        end
        
%         function Voc = map2Voc(matrix,tile)
%             
%             % get indoc from map
%             import Map.* 
%             indoc = map2indoc(matrix);
%             % get lococ from indoc
%             siz = size(matrix);
%             nc = length(indoc); % component number
%             lococ = cell(1,nc);
%             for i=1:nc
%                 lococ{i} = ind2loc(siz,tile,indoc{i});
%             end
%             % get Voc from lococ
%             Voc = cell(1,nc);
%             for i=1:nc
%                 Voc{i} = loc2vert(siz,tile,lococ{i});
%             end
%         end
        
        function Voc = indoc2Voc(siz,tile,indoc)
            
            import Map.*
            % get lococ from indoc
            nc = length(indoc); % component number
            lococ = cell(1,nc);
            for i=1:nc
                lococ{i} = ind2loc(siz,tile,indoc{i});
            end
            % get Voc from lococ
            Voc = cell(1,nc);
            for i=1:nc
                Voc{i} = loc2vert(siz,tile,lococ{i});
            end
        end
        
        function V = loc2vert(siz,tile,loc)
            % get V0 from Cuboid
            if length(siz)==2
                V0 = Cuboid2.tobvert(tile*ones(1,length(siz)));            
            else
                V0 = Cuboid.tobvert(tile*ones(1,length(siz)));
            end
            % get Voc from lococ
            nc = size(loc,1);
            V = loc;
            for i=1:nc
                V = [V;loc(i,:)+V0];
            end   
            V = V(convhull_(V),:);
        end
            
        function loc = ind2loc(siz,tile,ind)
            % get tile center coordinate from matrix index
            % - ind: matrix index (from 1 to size(matrix,1)*size(matrix,2)*size(matrix,3))
            % - siz: size of map matrix
            % - tile: scale of tiles
            % - loc: real coordinate of tile center at first quadrant 
            % |---------y
            % |              =>   map matrix
            % |x
            loc = zeros(length(ind),length(siz));
            for i=1:length(ind)
                if length(siz)==3
                    [irow,icol,ilay] = ind2sub(siz,ind(i));
                    sub = [irow,icol,ilay];
                else
                    [irow,icol] = ind2sub(siz,ind(i));
                    sub = [irow,icol];
                end
                lgrid = tile*ones(1,length(siz));
                loc(i,:) = (diag(lgrid)*(sub(:)-ones(length(lgrid),1))+lgrid(:)/2)';
            end
            % lx = lgrid(1); ly = lgrid(2); lz = lgrid(3);
            % x = (irow-1)*lx+lx/2;
            % y = (icol-1)*ly+ly/2;
            % z = (ilay-1)*lz+lz/2;
            % loc = [x,y,z];
        end
        
        function ind = loc2ind(siz,tile,loc)
            % get tile index from the real location 
            % (which tile are we located in?)
            % - ind: matrix index (from 1 to size(matrix,1)*size(matrix,2)*size(matrix,3))
            % - tile: scale of tiles
            % - loc: real coordinate of tile center at first quadrant 
            % |---------y
            % |              =>   map matrix
            % |x
            if isvec(loc)
                loc = loc(:)';
            end
            ind = zeros(1,size(loc,1));
            for i=1:size(loc,1)
                lgrid = tile*ones(1,length(siz));
                sub = (diag(lgrid)^-1*(loc(i,:)'-lgrid(:)/2)+ones(length(lgrid),1))';
                sub = round(sub); % ��������
                ind(i) = sub2ind_(siz,sub);
            end
        end
    end
end