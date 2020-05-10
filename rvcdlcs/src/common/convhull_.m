function I = convhull_(V)
% convhull_ - Convex hull
% 
%     This function returns the 2-D convex hull of the points V = [X,Y], where X
%     and Y are column vectors.
% 
%     I = convhull_(V)
%     I = convhull_(...,'simplify', logicalvar)

if iscolinear(V)
    if ~isempty(V)
        % ���ȫ�����߻���
        V1 = unique(V,'rows');
        % [C, ia, ic] = unique(A,'rows')
        % ��� A ���������� C = A(ia) �� A = C(ic)�� ��� A ��
        % ��������飬�� C = A(ia) �� A(:) = C(ic)�� ���ָ����
        % 'rows'ѡ��� C = A(ia,:) �� A = C(ic,:)�� ��� A ��
        % ���ʱ����� C = A(ia,:) �� A = C(ic,:)��
        Vc = sum(V1)/size(V1,1);
        V2 = round((V1-Vc)*10^-4)/10^-4; % V2ֻ�����ж�rank
    else
        V2 = [];
    end
    switch rank(V2)
        case 0 
            % �ж�Ϊ��
            if isempty(V2)
                I = [];
            else
                I = 1;
            end
        case 1
            if size(V1,1)==1
                % �ж�Ϊͬһ��
                I = 1;
            elseif size(V,1)==2
                % �ж�Ϊ����(2)
                I = [1;2;1];
            else
                % �ж�Ϊ����(>=3)
                I = furthest(V);
                I = [I(:);I(1)];
            end
        case 2
            % �ж�Ϊ����(��������ȫ������������ʾ)
            n = null(V1); % V*n==0˵������n��ֱ��V�е�ÿһ��Ԫ��
            o = (sum(V1)/size(V1,1))'; % oΪƽ��V��ԭ��
            z = n+o; x = V1(1,:)'-o; y = skew(z)*x; % x,y,zΪ��������ϵ��ƽ��VΪxoyƽ��
            g0 = [eye(3),zeros(3,1)]; g1 = [x(:),y(:),z(:),o(:)];
            T = e2h(g1)*e2h(g0)^-1;
            V3 = h2e(T^-1*e2h(V'))';
            I = convhull_(V3(:,1:2));          
        otherwise
            error('unknown size');
    end
else
    % MATLAB function
    I = convhull(V);
end
