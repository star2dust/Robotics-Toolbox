function gl_sdh = stdDH(dh)
theta = dh(1); d = dh(2); alpha = dh(3); a = dh(4);
gl_sdh = trotz(theta)*transl([0,0,d])*trotx(alpha)*transl([a,0,0]);
% cannot be too accurate
gl_sdh = [cos(theta) -sin(theta)*cos(alpha)  sin(theta)*sin(alpha)   a*cos(theta)
            sin(theta)  cos(theta)*cos(alpha)   -cos(theta)*sin(alpha)  a*sin(theta)
            0           sin(alpha)              cos(alpha)              d
            0           0                       0                       1];
end