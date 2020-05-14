function savelog(file,msg,opt)
if nargin<3
   opt = 'a'; 
end
fid = fopen(file, opt);
if fid == -1
  error('Cannot open log file.');
end
for i=1:size(msg,1)
    fprintf(fid, '%s: %s\n', datestr(now, 0), msg(i,:));
end
end