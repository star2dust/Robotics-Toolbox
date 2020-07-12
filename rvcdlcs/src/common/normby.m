function n = normby(V,s,p)
% normby - Vectors norms by row or column
% 
%     This function returns the Euclidean norm of vectors V.
%     s = 1 means by row, s = 2 means by column
%
%     n = norm(V,s)  
%     n = norm(V,s,p)
for i=1:size(V,s)
    if s==1
        if nargin<3
            n(i,:) = norm(V(i,:));
        else
            n(i,:) = norm(V(i,:),p);
        end
    elseif s==2
        if nargin<3
            n(:,i) = norm(V(:,i));
        else
            n(:,i) = norm(V(:,i),p);
        end
    else
        error('unknown argument');
    end
end