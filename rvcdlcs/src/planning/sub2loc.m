function loc = sub2loc(sub,lgrid)
% get real coordinate from row, column and layer subscripts of map matrix
% - sub:  row, column and layer subscripts of map matrix
% - lgrid: edge length of grid
% - loc: real coordinate of grid center at first quadrant
% |---------y  
% |              =>   map matrix
% |x
loc = (diag(lgrid)*(sub(:)-ones(length(lgrid),1))+lgrid(:)/2)';
end