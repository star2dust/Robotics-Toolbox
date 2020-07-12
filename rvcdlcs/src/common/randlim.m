function x = randlim(lim)
if size(lim,1)~=2||size(lim,3)~=1
   error('unknown size'); 
end
for i=1:size(lim,2)
    x(i) = lim(1,i)+rand(1)*diff(lim(:,i));
end
end