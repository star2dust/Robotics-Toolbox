function astardemo
%ASTARDEMO Demonstration of ASTAR algorithm
%
%   Copyright Bob L. Sturm, Ph. D., Assistant Professor
%   Department of Architecture, Design and Media Technology
%     formerly Medialogy
%   Aalborg University i Ballerup
%     formerly Aalborg University Copenhagen
%   $Revision: 0.1 $  $Date: 2011 Jan. 15 18h24:24$

n = 20;   % field size n x n tiles
wallpercent = 0.45;  % this percent of field is walls

% create the n x n FIELD with wallpercent walls containing movement costs, 
% a starting position STARTPOSIND, a goal position GOALPOSIND, the costs 
% A star will compute movement cost for each tile COSTCHART, 
% and a matrix in which to store the pointers FIELDPOINTERS
[field, startposind, goalposind, costchart, fieldpointers] = ...
  initializeField(n,wallpercent);

% initialize the OPEN and CLOSED sets and their costs
setOpen = [startposind]; setOpenCosts = [0]; setOpenHeuristics = [Inf];
setClosed = []; setClosedCosts = [];
movementdirections = {'R','L','D','U'};

% keep track of the number of iterations to exit gracefully if no solution
counterIterations = 1;

% create figure so we can witness the magic
axishandle = createFigure(field,costchart,startposind,goalposind);

% as long as we have not found the goal or run out of spaces to explore
while ~max(ismember(setOpen,goalposind)) && ~isempty(setOpen)
  % for the element in OPEN with the smallest cost
  [temp, ii] = min(setOpenCosts + setOpenHeuristics);
  % find costs and heuristic of moving to neighbor spaces to goal
  % in order 'R','L','D','U'
  [costs,heuristics,posinds] = findFValue(setOpen(ii),setOpenCosts(ii), ...
    field,goalposind,'euclidean');
  % put node in CLOSED and record its cost
  setClosed = [setClosed; setOpen(ii)];
  setClosedCosts = [setClosedCosts; setOpenCosts(ii)];
  % update OPEN and their associated costs
  if (ii > 1 && ii < length(setOpen))
    setOpen = [setOpen(1:ii-1); setOpen(ii+1:end)];
    setOpenCosts = [setOpenCosts(1:ii-1); setOpenCosts(ii+1:end)];
    setOpenHeuristics = [setOpenHeuristics(1:ii-1); setOpenHeuristics(ii+1:end)];
  elseif (ii == 1)
    setOpen = setOpen(2:end);
    setOpenCosts = setOpenCosts(2:end);
    setOpenHeuristics = setOpenHeuristics(2:end);
  else
    setOpen = setOpen(1:end-1);
    setOpenCosts = setOpenCosts(1:end-1);
    setOpenHeuristics = setOpenHeuristics(1:end-1);
  end
  % for each of these neighbor spaces, assign costs and pointers; 
  % and if some are in the CLOSED set and their costs are smaller, 
  % update their costs and pointers
  for jj=1:length(posinds)
    % if cost infinite, then it's a wall, so ignore
    if ~isinf(costs(jj))
      % if node is not in OPEN or CLOSED then insert into costchart and 
      % movement pointers, and put node in OPEN
      if ~max([setClosed; setOpen] == posinds(jj))
        fieldpointers(posinds(jj)) = movementdirections(jj);
        costchart(posinds(jj)) = costs(jj);
        setOpen = [setOpen; posinds(jj)];
        setOpenCosts = [setOpenCosts; costs(jj)];
        setOpenHeuristics = [setOpenHeuristics; heuristics(jj)];
      % else node has already been seen, so check to see if we have
      % found a better route to it.
      elseif max(setOpen == posinds(jj))
        I = find(setOpen == posinds(jj));
        % update if we have a better route
        if setOpenCosts(I) > costs(jj)
          costchart(setOpen(I)) = costs(jj);
          setOpenCosts(I) = costs(jj);
          setOpenHeuristics(I) = heuristics(jj);
          fieldpointers(setOpen(I)) = movementdirections(jj);
        end
      % else node has already been CLOSED, so check to see if we have
      % found a better route to it.
      else
        % find relevant node in CLOSED
        I = find(setClosed == posinds(jj));
        % update if we have a better route
        if setClosedCosts(I) > costs(jj)
          costchart(setClosed(I)) = costs(jj);
          setClosedCosts(I) = costs(jj);
          fieldpointers(setClosed(I)) = movementdirections(jj);
        end
      end
    end
  end
  if isempty(setOpen) break; end
  set(axishandle,'CData',[costchart costchart(:,end); costchart(end,:) costchart(end,end)]);
  % hack to make image look right
  set(gca,'CLim',[0 1.1*max(costchart(find(costchart < Inf)))]);
  drawnow; 
end

if max(ismember(setOpen,goalposind))
  disp('Solution found!');
  % now find the way back using FIELDPOINTERS, starting from goal position
  p = findWayBack(goalposind,fieldpointers);
  % plot final path
  plot(p(:,2)+0.5,p(:,1)+0.5,'Color',0.2*ones(3,1),'LineWidth',4);
  drawnow;
  drawnow;
  % celebrate
  [y,Fs] = audioread('wee.wav'); sound(y,Fs);
elseif isempty(setOpen)
  disp('No Solution!'); 
  [y,Fs] = audioread('pewee-ahh.wav'); 
  sound(y,Fs);
end
% end of the main function

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function p = findWayBack(goalposind,fieldpointers)
% This function will follow the pointers from the goal position to the
% starting position
    n = length(fieldpointers);  % length of the field
    posind = goalposind;
    % convert linear index into [row column]
    [py,px] = ind2sub([n,n],posind);
    % store initial position
    p = [py px];
    % until we are at the starting position
    while ~strcmp(fieldpointers{posind},'S')
      switch fieldpointers{posind}
        case 'L' % move left
          px = px - 1;
        case 'R' % move right
          px = px + 1;
        case 'U' % move up
          py = py - 1;
        case 'D' % move down
          py = py + 1;
      end
      p = [p; py px];
      % convert [row column] to linear index
      posind = sub2ind([n n],py,px);
    end
% end of this function

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [cost,heuristic,posinds] = findFValue(posind,costsofar,field, ...
  goalind,heuristicmethod)
% This function finds the movement COST for each tile surrounding POSIND in
% FIELD, returns their position indices POSINDS. They are ordered: right,
% left, down, up.
    n = length(field);  % length of the field
    % convert linear index into [row column]
    [currentpos(1) currentpos(2)] = ind2sub([n n],posind);
    [goalpos(1) goalpos(2)] = ind2sub([n n],goalind);
    % places to store movement cost value and position
    cost = Inf*ones(4,1); heuristic = Inf*ones(4,1); pos = ones(4,2);
    
    % if we can look left, we move from the right
    newx = currentpos(2) - 1; newy = currentpos(1);
    if newx > 0
      pos(1,:) = [newy newx];
      switch lower(heuristicmethod)
        case 'euclidean'
          heuristic(1) = abs(goalpos(2)-newx) + abs(goalpos(1)-newy);
        case 'taxicab'
          heuristic(1) = abs(goalpos(2)-newx) + abs(goalpos(1)-newy);
      end
      cost(1) = costsofar + field(newy,newx);
    end

    % if we can look right, we move from the left
    newx = currentpos(2) + 1; newy = currentpos(1);
    if newx <= n
      pos(2,:) = [newy newx];
      switch lower(heuristicmethod)
        case 'euclidean'
          heuristic(2) = abs(goalpos(2)-newx) + abs(goalpos(1)-newy);
        case 'taxicab'
          heuristic(2) = abs(goalpos(2)-newx) + abs(goalpos(1)-newy);
      end
      cost(2) = costsofar + field(newy,newx);
    end

    % if we can look up, we move from down
    newx = currentpos(2); newy = currentpos(1)-1;
    if newy > 0
      pos(3,:) = [newy newx];
      switch lower(heuristicmethod)
        case 'euclidean'
          heuristic(3) = abs(goalpos(2)-newx) + abs(goalpos(1)-newy);
        case 'taxicab'
          heuristic(3) = abs(goalpos(2)-newx) + abs(goalpos(1)-newy);
      end
      cost(3) = costsofar + field(newy,newx);
    end

    % if we can look down, we move from up
    newx = currentpos(2); newy = currentpos(1)+1;
    if newy <= n
      pos(4,:) = [newy newx];
      switch lower(heuristicmethod)
        case 'euclidean'
          heuristic(4) = abs(goalpos(2)-newx) + abs(goalpos(1)-newy);
        case 'taxicab'
          heuristic(4) = abs(goalpos(2)-newx) + abs(goalpos(1)-newy);
      end
      cost(4) = costsofar + field(newy,newx);
    end
    
    % return [row column] to linear index
    posinds = sub2ind([n n],pos(:,1),pos(:,2));
% end of this function

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [field, startposind, goalposind, costchart, fieldpointers] = ...
  initializeField(n,wallpercent)
% This function will create a field with movement costs and walls, a start
% and goal position at random, a matrix in which the algorithm will store 
% f values, and a cell matrix in which it will store pointers
    % create the field and place walls with infinite cost
    field = ones(n,n) + 10*rand(n,n);
    field(ind2sub([n n],ceil(n^2.*rand(floor(n*n*wallpercent),1)))) = Inf;
    % create random start position and goal position
    startposind = sub2ind([n,n],ceil(n.*rand),ceil(n.*rand));
    goalposind = sub2ind([n,n],ceil(n.*rand),ceil(n.*rand));
    % force movement cost at start and goal positions to not be walls
    field(startposind) = 0; field(goalposind) = 0;
    % put not a numbers (NaN) in cost chart so A* knows where to look 
    costchart = NaN*ones(n,n);
    % set the cost at the starting position to be 0
    costchart(startposind) = 0;
    % make fieldpointers as a cell array
    fieldpointers = cell(n,n);
    % set the start pointer to be "S" for start, "G" for goal
    fieldpointers{startposind} = 'S'; fieldpointers{goalposind} = 'G';
    % everywhere there is a wall, put a 0 so it is not considered
    fieldpointers(field == Inf) = {0};
% end of this function

%%%%%%%%%%%%%%%%%%%%  
function axishandle = createFigure(field,costchart,startposind,goalposind)
% This function creates a pretty figure
    % If there is no figure open, then create one
    if isempty(gcbf)
      f1 = figure('Position',[600 300 500 500],'Units','Normalized', ...
        'MenuBar','none');
      Caxes2 = axes('position', [0.01 0.01 0.98 0.98],'FontSize',12, ...
        'FontName','Helvetica');
    else
      % get the current figure, and clear it
      gcf; cla;
    end
    n = length(field);
    % plot field where walls are black, and everything else is white
    field(field < Inf) = 0;
    pcolor([1:n+1],[1:n+1],[field field(:,end); field(end,:) field(end,end)]);
    % set the colormap for the ploting the cost and looking really nice
    cmap = flipud(colormap('jet'));
    % make first entry be white, and last be black
    cmap(1,:) = zeros(3,1); cmap(end,:) = ones(3,1);
    % apply the colormap, but make red be closer to goal
    colormap(flipud(cmap));
    % keep the plot so we can plot over it
    hold on;
    % now plot the f values for all tiles evaluated
    axishandle = pcolor([1:n+1],[1:n+1],[costchart costchart(:,end); costchart(end,:) costchart(end,end)]);
    % plot goal as a yellow square, and start as a green circle
    [goalposy,goalposx] = ind2sub([n,n],goalposind);
    [startposy,startposx] = ind2sub([n,n],startposind);
    plot(goalposx+0.5,goalposy+0.5,'ys','MarkerSize',10,'LineWidth',6);
    plot(startposx+0.5,startposy+0.5,'go','MarkerSize',10,'LineWidth',6);
    % add a button so that can re-do the demonstration
    uicontrol('Style','pushbutton','String','RE-DO', 'FontSize',12, ...
      'Position', [1 1 60 40], 'Callback','astardemo');
 % end of this function 