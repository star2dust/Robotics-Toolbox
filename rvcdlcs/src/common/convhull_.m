function I = convhull_(V)
% Convex hull with relative interior
% 
%     This function returns the 2-D convex hull of the points V = [X,Y], where X
%     and Y are column vectors.
% 
%     I = convhull_(V)
%     I = convhull_(...,'simplify', logicalvar)

if iscolinear(V) % if V is colinear, we cannot use original MATLAB function
    if ~isempty(V) % An error of 0.001 can also be judged as collinear
        % remove duplicate points
        V1 = unique(V,'rows');
        % find central point
        Vc = sum(V1)/size(V1,1);
        V2 = round(V1-Vc,4); % use central as origin
    else
        V2 = [];
    end
    switch rank(V2)
        case 0 
            % if V = [] return []
            if isempty(V)
                I = [];
            else
                I = 1;
            end
        case 1
            if size(V,1)==1
                % if V contains 1 point
                I = 1;
            elseif size(V,1)==2
                % if V contains 2 points
                I = [1;2;1];
            else
                % if V contains more points(>=3)
                I = furthest(V);
                I = [I(:);I(1)];
            end
        case 2
            % the case with an error of 0.001 
            % e.g. V = [1.0003,2.0007;2.0005,4.0008;3.0005,6.0008]
            if size(V,2)==2
                I = furthest(V);
                I = [I(:);I(1)];
            else
                % a plane in 3d space 
                n = null(V1); % V*n==0 get the normal vector
                o = (sum(V1)/size(V1,1))'; % o is the central of V
                z = n+o; x = V1(1,:)'-o; y = skew(z)*x; % x,y,z as 3d axis
                g0 = [eye(3),zeros(3,1)]; g1 = [x(:),y(:),z(:),o(:)];
                T = e2h(g1)*e2h(g0)^-1;
                V3 = h2e(T^-1*e2h(V'))'; % back to 2d
                I = convhull_(V3(:,1:2));
            end
        otherwise
            % the case with an error of 0.001 
            % it is regarded as colinear but can be handled by convhull
            I = convhull(V);
    end
else
    % MATLAB function
    I = convhull(V);
end
