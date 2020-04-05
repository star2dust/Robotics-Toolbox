function I = convhull_(V)
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
                th = cart2pol(V1(:,1)-V1(1,1),V1(:,2)-V1(1,2));
                th_int = round(th(2:end)*100);
                if length(unique(th_int))==1
                    % 如果全部共线
                    I = furthest(V);
                    I = [I,I(1)];
                else
                    % 如果非全部共线也非同一点，则取凸包
                    I = convhull(V);
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
