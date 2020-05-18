function savevideo(dir,name,frame,siz)
initpath = which('startup.m');
respath = [initpath(1:end-length('startup.m')) dir];
if ~exist(respath,'dir')
    mkdir(respath);
end
videoname = increment([respath name],'.avi');
vdobj = VideoWriter(videoname);
open(vdobj);
for i=1:length(frame)
    if nargin==4
        frame(i).cdata = imresize(frame(i).cdata,siz);
    end
    writeVideo(vdobj,frame(i));
end
close(vdobj);
end