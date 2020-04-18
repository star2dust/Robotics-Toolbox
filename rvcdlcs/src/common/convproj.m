function [Vp,resnorm,exitflag] = convproj(V0,A,b,lim,Aeq,beq)

for i=1:size(V0,1)
    if isa(A,'cell')
        if nargin<6
            [x,resnorm(i),~,exitflag(i)]= lsqlin(eye(size(V0,2)),V0(i,:)',...
                A{i},b{i},[],[],lim{i}(1,:)',lim{i}(2,:)');
            
        else
            [x,resnorm(i),~,exitflag(i)] = lsqlin(eye(size(V0,2)),V0(i,:)',...
                A{i},b{i},Aeq{i},beq{i},lim{i}(1,:)',lim{i}(2,:)');
        end
    else
        if nargin<6
            [x,resnorm(i),~,exitflag(i)]  = lsqlin(eye(size(V0,2)),V0(i,:)',...
                A,b,[],[],lim(1,:)',lim(2,:)');
        else
            [x,resnorm(i),~,exitflag(i)]  = lsqlin(eye(size(V0,2)),V0(i,:)',...
                A,b,Aeq,beq,lim(1,:)',lim(2,:)');
        end
    end
    Vp(i,:) = x';
end
end