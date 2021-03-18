function v = element(x,ind)
% Get the element of x by index or subscript
if isa(ind,'cell')
    % subscript
    v = x(ind{:});
else
    % index
    v = x(ind);
end
end

