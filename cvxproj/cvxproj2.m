function [pj,ps] = cvxproj2(px,A,b)
% proj: calculate the projection of x to a convex set defined by Ax+b<=0 (2D)

% step 1: check if inside
if max(A*px+b)<0.00001
    pj=px;
else
    % step 2: get proj to edges
    for i=1:length(b)
        % normal & tangent vector
        vn(:,i) = A(i,:)';
        vt = rot2(pi/2)*vn(:,i);
        % calculate projection point
        pe(:,i) = [vn(:,i),vt]'\[-b(i);vt'*px];
    end
    % step 3: get proj to intersection   
    ps = [];
    for i=1:length(b)-1
        for j=i+1:length(b)
            % calculate intersection point
            ps = [ps,[vn(:,i),vn(:,j)]'\[-b(i);-b(j)]];
        end
    end
    % check if on the edge, get all candidate points
    i = 0;
    while i<length(pe(1,:))
        i=i+1;
        if max(A*pe(:,i)+b)>0.0001
            pe(:,i)=[];
            i=i-1;
        end
    end
    i = 0;
    while i<length(ps(1,:))
        i=i+1;
        if max(A*ps(:,i)+b)>0.0001
            ps(:,i)=[];
            i=i-1;
        end
    end
    % find the min distance point
    pes = [pe,ps];
    dis = 1000;
    for i=1:length(pes)
        if norm(px-pes(:,i))<dis
            pj = pes(:,i);
            dis = norm(px-pes(:,i));
        end
    end
    % order ps as patch vertices
    pth = atan2(ps(1,1)-ps(1,2:end),ps(2,1)-ps(2,2:end));
    [~,idx] = findNmin(pth,length(pth));
    ps = ps(:,[1,idx+1]);
end
