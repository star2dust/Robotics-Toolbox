clc;clear;

map=load('map2.txt');
start=[4,3]; goal=[16,18];
map_scalar=size(map,1);
%画栅格地图
figure
b = map;
b(end+1,end+1) = 0;
colormap([0 0 0;1 1 1]);
pcolor(0.5:size(map,2)+0.5,0.5:size(map,1)+0.5,b);%随机色彩
set(gca,'XTick',1:size(map,2),'YTick',1:size(map,1));
axis image ij; %沿每个坐标轴使用相同的数据单位，保持一致
hold on;

%标注起点和终点
scatter(start(2),start(1),'MarkerEdgeColor',[1 0 0],'MarkerFaceColor',[1 0 0], 'LineWidth',2);%start point
scatter(goal(2),goal(1),'MarkerEdgeColor',[0 1 0],'MarkerFaceColor',[0 1 0], 'LineWidth',2);%goal point
hold on;

[gx gy]=find(map==2); %找到所有门的位置
gate=[gx gy];
str=1:1:size(gate,1);
%画出门的位置
scatter(gate(:,2),gate(:,1),400,'y','s','filled'); %scatter(y,x,size,颜色,形状,填充)
hold on;
%给门标序号
for i=1:size(gate)
    str=num2str(i); %数字转字符串
    text(gate(i,2),gate(i,1),str,'FontSize',10);
    hold on;
end