function inobst=inobstpoly(start,goal,envir,disp_base_min)
% inobstpoly    Collision checking for start, goal and line segment bewteen them.

% add nan between obstacles
obstpoly = envir.obstacle{1}';
for i=2:length(envir.obstacle)
    obstpoly = [obstpoly,NaN(2,1),envir.obstacle{i}'];
end
% inflate point to a polygon
dist = 0.5;
disp_ind = convhull_(disp_base_min);
disp_base_diff = diff(disp_base_min(disp_ind,:));
disp_base_inflate = [];
for i=1:size(disp_base_min)
   disp_base_inflate = [disp_base_inflate;disp_base_min(disp_ind(i),:)+(0:dist:norm(disp_base_diff(i,:)))'.*unit(disp_base_diff(i,:))];
end
% check if goal is feasible
if max(inpolygon(goal(1)+disp_base_inflate(:,1),goal(2)+disp_base_inflate(:,2),obstpoly(1,:),obstpoly(2,:)))
    inobst = true;
elseif norm(goal-start)>10^-4
    pos_line = start+(0:dist:norm(goal-start))'.*unit(goal-start);
    disp_base_line = [];
    for i=1:size(pos_line,1)
        disp_base_line = [disp_base_line;disp_base_inflate+pos_line(i,1:2)];
    end
    % check if posline is feasible
    if max(inpolygon(disp_base_line(:,1),disp_base_line(:,2),obstpoly(1,:),obstpoly(2,:)))
        inobst = true;
    else
        inobst = false;
    end
else
    inobst = false;
end