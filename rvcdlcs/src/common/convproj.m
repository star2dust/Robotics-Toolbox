function [Vp,resnorm,exitflag] = convproj(V0,A,b,lim,Aeq,beq)


Vp = V0;
resnorm = zeros(size(V0,1),1);
exitflag = ones(size(V0,1),1);
for i=1:size(V0,1)
    if isa(A,'cell')
        if nargin<6
            [x,rn,~,ef]= lsqlin(eye(size(V0,2)),V0(i,:)',...
                A{i},b{i},[],[],lim{i}(1,:)',lim{i}(2,:)');
            
        else
            [x,rn,~,ef] = lsqlin(eye(size(V0,2)),V0(i,:)',...
                A{i},b{i},Aeq{i},beq{i},lim{i}(1,:)',lim{i}(2,:)');
        end
    else
        if nargin<6
            [x,rn,~,ef]  = lsqlin(eye(size(V0,2)),V0(i,:)',...
                A,b,[],[],lim(1,:)',lim(2,:)');
        else
            [x,rn,~,ef]  = lsqlin(eye(size(V0,2)),V0(i,:)',...
                A,b,Aeq,beq,lim(1,:)',lim(2,:)');
        end
    end
    Vp(i,:) = x';
    resnorm(i,:) = rn;
    exitflag(i,:) = ef;
end
end