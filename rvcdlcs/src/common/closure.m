function I = closure(V)
% generate the closure of vertices

if isempty(V)||~ismatrix(V)
    I = [];
else
    V1 = unique(V,'rows');
    if size(V1,1)==1
        % 判断是否为同一点
        I = 1;
    else
        switch size(V1,2)
            case 2
                % 判断是否全部共线
                A = cart2pol(V1(:,1)-V1(1,1),V1(:,2)-V1(1,2));
                th_int = ceil(A(2:end)*100);
                if length(unique(th_int))==1
                    % 如果全部共线
                    I = furthest(V);
                    I = [I,I(1)];
                else
                    % 如果非全部共线也非同一点，则取最外围顶点
                    Vc = sum(V)/size(V,1); % 中心点
                    [A,R] = cart2pol(V(:,1)-Vc(1),V(:,2)-Vc(2)); % 极坐标
                    % 极坐标按R降序排列
                    [~,ir] = sort(R,'descend');
                    A = round(A(ir)*100);
                    % [C, ia, ic] = unique(A,'rows')
                    % 如果 A 是向量，则 C = A(ia) 且 A = C(ic)。 如果 A 是
                    % 矩阵或数组，则 C = A(ia) 且 A(:) = C(ic)。 如果指定了 
                    % 'rows'选项，则 C = A(ia,:) 且 A = C(ic,:)。 如果 A 是
                    % 表或时间表，则 C = A(ia,:) 且 A = C(ic,:)。
                    [~,ia,~] = unique(A,'rows','first');
                    I = 1:size(V,1);
                    Ir = I(ir); Ia = Ir(ia);
                    I = [Ia(:)',Ia(1)];
                end
            case 3
                % 判断是否全部共面
                warning('3D convhull features to be updated');
                I = convhull(V);
            otherwise
                error('unknown size');
        end
    end
end