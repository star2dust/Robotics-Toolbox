function vertout = dilate(vertin,radius)
% DILATE  Dilate obstacles with a given radius
x = [];
for i=1:size(vertin,1)
    x = [x;circle_(vertin(i,:),radius)];
end
vertout = x(convhull_(x),:);
end