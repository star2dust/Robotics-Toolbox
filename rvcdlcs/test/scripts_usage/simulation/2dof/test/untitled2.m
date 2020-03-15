close all
clear

[x,y] = meshgrid(-3:.1:3,-3:.1:3);
z = [x;y];
a = 1; b = 1; c = 1;
f = a*z.^2+b*z+c;
surf(x,y,f);

% [th2,th3] = meshgrid(-pi:0.1:0,-pi:0.1:0);
% y = cos(th2) - cos(2*th2 + th3) - cos(2*th2)/2 - cos(2*th3) - cos(2*th2 + 2*th3) - cos(th2 + 2*th3) + cos(th3)+5/2;
% c1 = max(-th2-th3-pi/2,th2+th3);
% c2 = max(-th2-pi/2,th2);
% cons = max(c1,c2);
% th2(cons>0)=nan;
% th3(cons>0)=nan;
% surf(th2,th3,sqrt(y))
% ylabel('Y');xlabel('X');zlabel('Z');
% 
% m = 3;
% dx = sym('dx',[m,1],'real');
% dy = sym('dy',[m,1],'real');
% % calculate
% ls = sym('ls',[m,1],'real');
% lc = sym('lc',[m,1],'real');
% th = sym('th',[m,1],'real');
% l = sym('l',[m,1],'real');
% l = ones(m,1);
% % lsin and lcos
% for i=1:m
%     for j=1:m
%         ls(i) = l(i)*sin(sum(th(1:i)));
%         lc(i) = l(i)*cos(sum(th(1:i)));
%     end
% end
% 
% % dxdth and dydth
% for i=1:m
%     dx(i) = 0;
%     dy(i) = 0;
%     for j=i:m
%         dx(i) = dx(i)-ls(j);
%         dy(i) = dy(i)+lc(j);
%     end
% end
% 
% mupow2 = (dx'*dx)*(dy'*dy)-(dx'*dy)^2;
% simplify(mupow2)
% 
% 
% % constraints
% [th2, th3] = meshgrid(-pi/2:.02:0,-pi/2:.02:0);
% c1 = max(-th2-th3-pi/2,th2+th3);
% c2 = max(-th2-pi/2,th2);
% c3 = max(-th3-pi/2,th3-pi/2);
% c4 = max(2*th2 + th3+pi/2,-2*th2 - th3-pi);
% c6 = max(2*th2+pi/2,-2*th2-pi);
% c8 = max(2*th3+pi/2,-2*th3-pi);
% c10 = max(2*th2 + 2*th3+pi/2,-2*th2 - 2*th3-pi);
% c12 = max(th2 + 2*th3+pi/2,-th2 - 2*th3-pi);
% cons = max(c1,max(c2,max(c3,max(c4,max(c6,max(c8,max(c10,c12)))))));
% th2(cons>0)=nan;
% th3(cons>0)=nan;
% plot(th2(:),th3(:),'y.')