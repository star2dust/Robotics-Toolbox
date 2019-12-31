classdef MobileRevolute < handle
    properties
        name
        base % Cuboid
        arm % SerialLink
    end
    
    methods
        function mr = MobileRevolute(edge,twist,opt)
            % MobileRevolute creates m-dof MobileRevolute robot object
            % opt statement
            opt.name = [];
            % opt parse: only stated fields are chosen to opt, otherwise to arg
            [opt,arg] = tb_optparse(opt, varargin); 
            obj.name = opt.name; 
            % argument parse
            if length(arg)==2
                edge = arg{1};
                twist = arg{2};
            else
                error('unknown arguments');
            end
            % edge,twist,bq,aq
            mr.base = Cuboid()
        end
    end
end