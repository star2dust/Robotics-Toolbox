function F = SE3toPose(SE3)
% p<pi/2 return [r,p,y]; p>pi/2 return [r-pi,pi-p,y-pi]
% p=pi/2 or -pi/2 singular; 
for i=1:length(SE3)
    F(:,i) = [SE3(i).t;SE3(i).torpy'];
end
end