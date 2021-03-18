function savevideo(name,frame,siz)
% Save frames into video files (avi)
name = increment(name,'.avi');
vdobj = VideoWriter(name);
open(vdobj);
for i=1:length(frame)
    if ~isempty(frame(i).cdata)
        if nargin==3
            frame(i).cdata = imresize(frame(i).cdata,siz);
        end
        writeVideo(vdobj,frame(i));
    end
end
close(vdobj);
end

function name = increment(name,type)
ctr = 0;
while exist([name type],'file')
    ctr = ctr+1;
    if ctr==1
        name = [name num2str(ctr)];
    else
        name = [name(1:end-length(num2str(ctr))) num2str(ctr)];
    end
end
end