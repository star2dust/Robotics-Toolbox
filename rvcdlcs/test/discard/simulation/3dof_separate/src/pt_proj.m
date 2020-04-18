function pj = pt_proj(px,ur,dl)
pj = (px-ur>0).*ur+(px-dl<0).*dl+(px<=ur).*(px>=dl).*px;
% test
% figure
% vert = [ur,[ur(1);dl(2)],dl,[dl(1);ur(2)]];
% patch(vert(1,:),vert(2,:),'y')
% axis equal
% hold on
% plot(px(1),px(2),'bo');
% plot(pj(1),pj(2),'ro');
end