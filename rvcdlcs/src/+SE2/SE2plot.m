function h = SE2plot(g,C,S)
subplot(1,1,1);
g_SE2 = SE2(g);
h = g_SE2.plot;
if nargin>1
    for i = 1:length(h.Children)
        h.Children(i).Color = C(1);
        if length(C)>1
            h.Children(i).LineStyle = C(2:end);
        end
        if nargin>2
            if i<=2
                h.Children(i).Position = h.Children(i).Position*S;
            else
                h.Children(i).XData = h.Children(i).XData*S;
                h.Children(i).YData = h.Children(i).YData*S;
            end
        end
    end
end
end