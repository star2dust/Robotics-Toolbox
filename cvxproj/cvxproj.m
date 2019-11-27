function [pj,ps] = cvxproj(px,A,b)
% proj: calculate the projection of x to a convex set defined by Ax+b<=0 (2D,3D)

% step 1: check if inside
if max(A*px+b)<0.00001
    pj=px;
else
    % step 2: get proj to edges(2D)/faces(3D)
    for i=1:length(b)
        if length(px)<3
            % normal & tangent vector
            vn(:,i) = A(i,:)';
            vt = rot2(pi/2)*vn(:,i); % so(n)
            % calculate projection point
            pe(:,i) = [vn(:,i),vt]'\[-b(i);vt'*px];
        else
            % normal & tangent vector
            vn(:,i) = A(i,:)';
            vz = [0;0;1];
            % vz to vn => rotation angle & normalized axis 
            th_zn = acos(vn(:,i)'*vz/(norm(vn(:,i)*norm(vz))));
            w_zn = skew(vz)*vn(:,i); % so(n)
            w_zn = w_zn/norm(w_zn);
            % Rodrigues' rotation formula
            R_zn = expm(skew(w_zn)*th_zn); % SO(n)
            % vn frame
            vt = R_zn(:,1:end-1);
            % calculate projection point
            pe(:,i) = [vn(:,i),vt]'\[-b(i);vt'*px];
        end
    end
    % step 2.1: get proj to intersection edges(3D)
    if length(px)>=3
        comb = nchoosek(1:length(b),2);
        for i=1:size(comb,1)
            vnn = skew(vn(:,comb(i,1)))*vn(:,comb(i,2));
            pe(:,i+length(b)) = [vn(:,comb(i,:)),vnn]'\[-b(comb(i,:));vnn'*px];
        end
    end
    % step 3: get proj to intersection points
    comb = nchoosek(1:length(b),length(px));
    for i=1:size(comb,1)
        ps(:,i) = vn(:,comb(i,:))'\-b(comb(i,:));
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
