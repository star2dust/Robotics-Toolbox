function pj = circproj(px,cen,rad)
vec = px-cen;
pj = vec/norm(vec)*rad+cen;
% test
% figure
% th = 0:.1:2*pi;
% x = rad*cos(th)+cen(1);
% y = rad*sin(th)+cen(2);
% plot(x,y,'y');
% axis equal
% hold on
% plot(px(1),px(2),'bo');
% plot(pj(1),pj(2),'ro');
end