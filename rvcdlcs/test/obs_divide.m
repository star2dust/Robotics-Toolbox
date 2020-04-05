function [oql, oqr] = obs_divide(oq, oind, dir, lgrid)
tile = Cuboid2(lgrid);
oql = []; oqr = [];
for i=1:length(oind)
    if skew2(dir)*tile.vert(oq(i,:))'>=0
        oql = [oql;tile.vert(oq(i,:))];
    else
        oqr = [oqr;tile.vert(oq(i,:))];
    end
end
kl = convhull(oql(:,1),oql(:,2));
kr = convhull(oqr(:,1),oqr(:,2));
oql = oql(kl,:);
oqr = oqr(kr,:);
end