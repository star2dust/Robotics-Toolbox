function xi = vlog(H)
    xi = SE3.hatvee(logm(H));
    xi = [xi(4:6);xi(1:3)];
end