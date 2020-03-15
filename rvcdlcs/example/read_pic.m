close all
clear

f=imread('01.png');
figure(1);
imshow(f);
title('原图');

figure(2);
bw1=imbinarize(f);%使用默认值0.5
imshow(bw1(:,:,1))
title('使用0.5作为门槛时的二值图像');

figure(3);
level=graythresh(f);%使用graythresh计算灰度门槛
bw2=imbinarize(f,level);
imshow(bw2(:,:,1));
title('通过graythresh计算灰度门槛时的二值图像');