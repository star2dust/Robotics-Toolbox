function ind = sub2ind_(siz,sub)
subsplit = num2cell(sub,1);
ind = sub2ind(siz,subsplit{:});
end