function sub = ind2sub_(siz,ind)
% Subscripts from linear index
% 
%     This function returns the matrices SUB = [I1,I2,...] containing the equivalent row
%     and column subscripts corresponding to each linear index in the matrix IND for a
%     matrix of size siz.
% 
%     SUB = ind2sub_(siz,IND)

ind = ind(:);
[sub{1:length(siz)}]= ind2sub(siz,ind);
sub = [sub{:}];
end