function out = circle_(cen, rad, varargin)
% CIRCLE_ Compute points on a circle 
% first point in the direction of 'pd'

opt.d = [1,0];
[opt,arglist] = tb_optparse(opt, varargin);

if nargout > 0
    % return now
    p = circle(zeros(1,2), rad, arglist{:});
    thd = cart2pol(opt.d(1),opt.d(2));
    out = (SE2([cen,thd])*p)';
    out = [out;out(1,:)];
    return;
else
    % else plot the circle
    circle(cen, rad, arglist{:});
end

end