function [I,V] = boundary_(V,s)
% boundary_ - Boundary of a set of points in 2-D or 3-D
%
%     This MATLAB function returns a vector of point indices representing a single
%     conforming 2-D boundary around the points (x,y).
%
%     k = boundary(V)
%     k = boundary(___,s) (s=0, convex hull; s=1, compact boundary)
%     [k,v] = boundary(___)

if iscolinear(V)
    if ~isempty(V)
        % 如果全部共线或共面
        V1 = unique(V,'rows');
        % [C, ia, ic] = unique(A,'rows')
        % 如果 A 是向量，则 C = A(ia) 且 A = C(ic)。 如果 A 是
        % 矩阵或数组，则 C = A(ia) 且 A(:) = C(ic)。 如果指定了
        % 'rows'选项，则 C = A(ia,:) 且 A = C(ic,:)。 如果 A 是
        % 表或时间表，则 C = A(ia,:) 且 A = C(ic,:)。
        Vc = sum(V1)/size(V1,1);
        V2 = round((V1-Vc)*100)/100;
    else
        V2 = V;
    end
    switch rank(V2)
        case 0
            % 判断为空
            I = V;
        case 1
            if size(V1,1)==1
                % 判断为同一点
                I = 1;
            elseif size(V,1)==2
                % 判断为共线(2)
                I = [1;2;1];
            else
                % 判断为共线(>=3)
                I = furthest(V);
                I = [I(:);I(1)];
            end
        case 2
            % 判断为共面
            % 以下向量全部用列向量表示
            n = null(V1); % V*n==0说明向量n垂直于V中的每一个元素
            o = (sum(V1)/size(V1,1))'; % o为平面V的原点
            z = n+o; x = V1(1,:)'-o; y = skew(z)*x; % x,y,z为正交坐标系，平面V为xoy平面
            g0 = [eye(3),zeros(3,1)]; g1 = [x(:),y(:),z(:),o(:)];
            T = e2h(g1)*e2h(g0)^-1;
            V3 = h2e(T^-1*e2h(V'))';
            I = boundary_(V3(:,1:2));          
        otherwise
            error('unknown size');
    end
else
    % MATLAB function
    if nargin<2
        I = boundary(V);
    else
        I = boundary(V,s);
    end
end
end