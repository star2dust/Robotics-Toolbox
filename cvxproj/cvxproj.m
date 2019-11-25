function [pj,ps] = proj(px,A,b)
% proj: calculate the projection of x to a convex set defined by Ax+b<=0

% step 1: check if inside
if max(A*px+b)<0.00001
    pj=px;
else
    % vector from x to proj
    syms x y real
    vx = px-[x;y];
    % step 2: get proj to edges
    pe = [];
    for i=1:length(b)
        k(i) = -A(i,1)/A(i,2);
        if k(i)==inf
            vc = [0;1];
            r.y = solve(vx'*vc);
            r.x = solve(A(i,:)*[x;y]+b(i,:));
            pe = [pe,[r.x;eval(r.y)]];
        else
            if k(i)==0
                vc = [1;0];
                r.x = solve(vx'*vc);
                r.y = solve(A(i,:)*[x;y]+b(i,:));
                pe = [pe,[eval(r.x);r.y]];
            else
                vc = [1;k(i)];
                r = solve(vx'*vc,A(i,:)*[x;y]+b(i,:));
                pe = [pe,[eval(r.x);eval(r.y)]];
            end
        end
    end
    % step 3: get proj to intersection
    ps = [];    
    for i=1:length(b)-1
        for j=i+1:length(b)
            if ~((k(i)==0&&k(j)==0)||(k(i)==inf&&k(j)==inf))
                if k(i)==inf           
                    r.x = solve(A(i,:)*[x;y]+b(i,:));
                    r.y = solve(A(j,:)*[r.x;y]+b(j,:));
                    ps = [ps,[eval(r.x);eval(r.y)]];
                else
                    if k(i)==0
                        r.y = solve(A(i,:)*[x;y]+b(i,:));
                        r.x = solve(A(j,:)*[x;r.y]+b(j,:));
                        ps = [ps,[eval(r.x);eval(r.y)]];
                    else
                        if k(j)==inf
                            r.x = solve(A(j,:)*[x;y]+b(j,:));
                            r.y = solve(A(i,:)*[r.x;y]+b(i,:));
                            ps = [ps,[eval(r.x);eval(r.y)]];
                        else
                            if  k(j)==0
                                r.y = solve(A(j,:)*[x;y]+b(j,:));
                                r.x = solve(A(i,:)*[x;r.y]+b(i,:));
                                ps = [ps,[eval(r.x);eval(r.y)]];
                            else
                                r = solve(A(i,:)*[x;y]+b(i,:),A(j,:)*[x;y]+b(j,:));
                                ps = [ps,[eval(r.x);eval(r.y)]];
                            end
                        end
                    end
                end 
            end
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
