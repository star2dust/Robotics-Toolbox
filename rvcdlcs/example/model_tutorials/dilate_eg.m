close all
clear

import Map.*

m1 = zeros(9,10);
m1(4:6,4:7) = 1;
imshow(~imresize(m1,40,'nearest'));

m2 = dilateMap(m1,'sphere',1);
figure
imshow(~imresize(m2,40,'nearest'));