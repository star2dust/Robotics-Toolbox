function sub = loc2sub(loc,lgrid)
% get row, column and layer subscripts in map matrix from real coordinate
% - loc: real coordinate of grid center at first quadrant
% - lgrid: edge length of grid
% - sub:  row and column subscripts of map matrix
% |---------y  
% |              =>   map matrix
% |x

sub = (diag(lgrid)^-1*(loc(:)-lgrid(:)/2)+ones(length(lgrid),1))'; 
end