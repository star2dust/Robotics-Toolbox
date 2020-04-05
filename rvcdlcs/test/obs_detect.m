function [oq, oind] = obs_detect(oq,oind,rloc,drad,map,lgrid)
% detect obstacles from a map matrix
size_map = size(map);
dsub = loc2sub(rloc,lgrid);
dsub_min = dsub; dsub_max = dsub; 
dsub_min(1:2) = max(floor(dsub_min(1:2)-drad/min(lgrid)),0*size(map));
dsub_max(1:2) = min(ceil(dsub_max(1:2)+drad/min(lgrid)),size(map));
dind_min = sub2ind(size_map,dsub_min(1),dsub_min(2));
dind_max = sub2ind(size_map,dsub_max(1),dsub_max(2));
ind = [];
for i=1:length(oind)
    if oind(i)<=dind_max&&oind(i)>=dind_min
        if norm(oq(i,1:2)-rloc)<=drad
            ind = [ind,i];
        end
    end
end
oind = oind(ind);
oq = oq(ind,:);
% tile = Cuboid2(lgrid); obs = [];
% for i=1:length(dind)
%     obs = [obs;tile.vert(dq(i,:))];
% end
% obs_l = []; obs_r = [];
% for i=1:length(dind)
%     if wedge2(dir)*tile.vert(dq(i,:))'>=0
%         obs_l = [obs_l;tile.vert(dq(i,:))];
%     else
%         obs_r = [obs_r;tile.vert(dq(i,:))];
%     end
% end
% kl = convhull(obs_l(:,1),obs_l(:,2));
% kr = convhull(obs_r(:,1),obs_r(:,2));
% obsl = obs_l(kl,:);
% obsr = obs_r(kr,:);
end