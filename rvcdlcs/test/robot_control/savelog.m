function savelog(file,msg)
fid = fopen(file, 'a');
if fid == -1
  error('Cannot open log file.');
end
fprintf(fid, '%s: %s\n', datestr(now, 0), msg);
end