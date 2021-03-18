function ind = sub2ind_(siz,sub)
% Convert subscripts to linear indices
% 
%     This MATLAB function returns the linear index equivalents to the row and column
%     subscripts rowSub and colSub for a matrix of size matrixSize.
% 
%     Ind = sub2ind_(matrixSize, Sub)
        
subsplit = num2cell(sub,1);
ind = sub2ind(siz,subsplit{:});
end