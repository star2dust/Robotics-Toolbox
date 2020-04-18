function [map, oq, oind] = obs_read(str,lgrid)
f1=imread(str);
bw1=imbinarize(f1);%使用默认值0.5
map = ~bw1(:,:,1);
[oq, oind] = map2cub(map,lgrid);
end