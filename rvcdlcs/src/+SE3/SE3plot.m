function h = SE3plot(SE3,C)
subplot(1,1,1);
h = SE3.plot;
if nargin>1
    for i = 1:length(h.Children)
        h.Children(i).Color = C(1);
        if length(C)>1
            h.Children(i).LineStyle = C(2:end);
        end
    end
end
end