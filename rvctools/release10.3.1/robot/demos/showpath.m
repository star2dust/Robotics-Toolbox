dxf = DXform(occgrid);
dxf.plan(goal);
pth = dxf.query(start)

pth = [start; pth; goal];

anim = Animate('path.mp4', 'fps', 1);

for p=pth'
    showpixels(cost, 'contrast', 6, 'fmt', '%.2g', 'cscale', [0 12], 'fontsize', 20, 'infsymbol', 'nancolor', 'nohidenan', 'nohideinf', 'infcolor')
        xlabel('x', 'FontSize', 20); ylabel('y', 'FontSize', 20)

    plot_circle(p, 0.4, 'fillcolor', 'y', 'edgecolor', 'None', 'alpha', 0.6)
    anim.add();
end
anim.close()