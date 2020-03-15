function loc = sub2loc(sub,lgrid)
% get real coordinate from map index
% - sub:  row and column subscripts of map matrix
% - lgrid: edge length of grid
% - loc: real coordinate of grid center at first quadrant
% |---------y  
% |              =>   map matrix
% |x
loc = (diag(lgrid)*(sub(:)-ones(length(lgrid),1))+lgrid(:)/2)';
end