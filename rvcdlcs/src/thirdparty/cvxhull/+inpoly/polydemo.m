function polydemo(varargin)
%POLYDEMO run a series of point-in-polygon demos.
%   POLYDEMO(II) runs the II-th demo, where +1 <= II <= +3. 
%   Demo problems illustrate varying functionality for the 
%   INPOLY2 routine.
%
%   See also INPOLY2, INPOLYGON

%   Darren Engwirda : 2018 --
%   Email           : darren.engwirda@columbia.edu
%   Last updated    : 28/10/2018

    if (nargin>=1) 
        id = varargin{1}; 
    else
        id =   +1;
    end

%------------------------------------------------- call demo
    switch (id)
        case 1, demo1;
        case 2, demo2;
        case 3, demo3;
        
        otherwise
    error('polydemo:invalidInput','Invalid demo selection.');
    end

end

function demo1
%-----------------------------------------------------------
    fprintf(1,[...
'   INPOLY2 provides fast point-in-polygon queries for ob-\n',... 
'   jects in R^2. Like INPOLYGON, it detects points inside\n',...
'   and on the boundary of polygonal geometries.\n\n']);

    filename = mfilename('fullpath');
    filepath = fileparts( filename );

    addpath([filepath,'/mesh-file']);

    node = [
        4,0         % outer nodes
        8,4
        4,8
        0,4
        3,3         % inner nodes
        5,3
        5,5
        3,5
        ] ;
    edge = [
        1,2         % outer edges
        2,3
        3,4
        4,1
        5,6         % inner edges
        6,7
        7,8
        8,5
        ] ;

   [xpos,ypos] = meshgrid(-1.:0.2:+9.) ;
    xpos = xpos(:) ;
    ypos = ypos(:) ;

   [stat,bnds] = ...
        inpoly2([xpos,ypos],node,edge) ; 

    figure;
    plot(xpos(~stat),ypos(~stat),'r.', ...
        'markersize',14) ;
    axis equal off; hold on;
    plot(xpos( stat),ypos( stat),'b.', ...
        'markersize',14) ;
    plot(xpos( bnds),ypos( bnds),'ks') ;

end

function demo2
%-----------------------------------------------------------
    fprintf(1,[...
'   INPOLY2 supports multiply-connected geometries, consi-\n',... 
'   sting of arbitrarily nested sets of outer + inner bou-\n',...
'   ndaries.\n\n']);

    filename = mfilename('fullpath');
    filepath = fileparts( filename );

    addpath([filepath,'/mesh-file']);

    geom = loadmsh( ...
        [filepath,'/test-data/lakes.msh']);
    
    node = geom.point.coord(:,1:2);
    edge = geom.edge2.index(:,1:2);

    emid = .5 * node(edge(:,1),:) ...
         + .5 * node(edge(:,2),:) ;
    
    half = max (node,[],1) ...
         + min (node,[],1) ;
    half = half * +0.5 ;
    scal = max (node,[],1) ...
         - min (node,[],1) ;

    rpts = rand(2500,2) ;
    rpts(:,1) = ...
    1.10*scal(1)*(rpts(:,1)-.5)+half(1);
    rpts(:,2) = ...
    1.10*scal(2)*(rpts(:,2)-.5)+half(2);

    xpos = [ ...
    node(:,1); emid(:,1); rpts(:,1)] ;
    ypos = [ ...
    node(:,2); emid(:,2); rpts(:,2)] ;
    
   [stat,bnds] = ...
        inpoly2([xpos,ypos],node,edge) ; 

    figure;
    plot(xpos(~stat),ypos(~stat),'r.', ...
        'markersize',14) ;
    axis equal off; hold on;
    plot(xpos( stat),ypos( stat),'b.', ...
        'markersize',14) ;
    plot(xpos( bnds),ypos( bnds),'ks') ;

end

function demo3
%-----------------------------------------------------------
    fprintf(1,[...
'   INPOLY2 implements a "pre-sorted" variant of the cros-\n',...
'   sing-number test - returning queries in approximately \n',...
'   O((N+M)*LOG(N)) time for configurations consisting of \n',...
'   N test points and M edges. This is often considerably \n',...
'   faster than conventional approaches that typically re-\n',...
'   quire O(N*M) operations.\n\n']);

    filename = mfilename('fullpath');
    filepath = fileparts( filename );

    addpath([filepath,'/mesh-file']);

    geom = loadmsh( ...
        [filepath,'/test-data/coast.msh']);
    
    node = geom.point.coord(:,1:2);
    edge = geom.edge2.index(:,1:2);

    emid = .5 * node(edge(:,1),:) ...
         + .5 * node(edge(:,2),:) ;
    
    half = max (node,[],1) ...
         + min (node,[],1) ;
    half = half * +0.5 ;
    scal = max (node,[],1) ...
         - min (node,[],1) ;

    rpts = rand(2500,2) ;
    rpts(:,1) = ...
    1.10*scal(1)*(rpts(:,1)-.5)+half(1);
    rpts(:,2) = ...
    1.10*scal(2)*(rpts(:,2)-.5)+half(2);

    xpos = [ ...
    node(:,1); emid(:,1); rpts(:,1)] ;
    ypos = [ ...
    node(:,2); emid(:,2); rpts(:,2)] ;
    
    tic
   [stat,bnds] = ...
        inpoly2([xpos,ypos],node,edge) ;
    fprintf(1,'Runtime: %f (INPOLY2)  \n',toc);
    
    tic
   [stat,bnds] = inpolygon( ...
        xpos,ypos,node(:,1),node(:,2)) ;
    fprintf(1,'Runtime: %f (INPOLYGON)\n',toc);

    figure;
    plot(xpos(~stat),ypos(~stat),'r.', ...
        'markersize',14) ;
    axis equal off; hold on;
    plot(xpos( stat),ypos( stat),'b.', ...
        'markersize',14) ;
    plot(xpos( bnds),ypos( bnds),'ks') ;

end



