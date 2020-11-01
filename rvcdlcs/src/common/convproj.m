function [xf,resnorm,exitflag] = convproj(x0,A,b,lim,Aeq,beq)


resnorm = zeros(size(x0,1),1);
exitflag = ones(size(x0,1),1);
if size(x0,1)==1||size(x0,2)==1
   x0 = x0(:)'; 
end
xf = x0;
for i=1:size(x0,1)
    if isa(A,'cell')
        % multiple vectors for multiple constraint sets
        if nargin==4
            [x,rn,~,ef]= lsqlin(eye(size(x0,2)),x0(i,:)',...
                A{i},b{i},[],[],lim{i}(1,:)',lim{i}(2,:)');
        elseif nargin<4
            [x,rn,~,ef] = lsqlin(eye(size(x0,2)),x0(i,:)',A{i},b{i});
        else
            [x,rn,~,ef] = lsqlin(eye(size(x0,2)),x0(i,:)',...
                A{i},b{i},Aeq{i},beq{i},lim{i}(1,:)',lim{i}(2,:)');
        end
    elseif isempty(A)&&isempty(b)
        for j=1:length(x0(i,:))
            if x0(i,j)>lim(2,j)
                x0(i,j)=lim(2,j);
            end
            if x0(i,j)<lim(1,j)
                x0(i,j)=lim(1,j);
            end
            x = x0(i,:)';
            rn = 0;
            ef = 0;
        end
    else
        % multiple vectors for the same constraint set
        if nargin==4
            [x,rn,~,ef]  = lsqlin(eye(size(x0,2)),x0(i,:)',...
                A,b,[],[],lim(1,:)',lim(2,:)');
        elseif nargin<4
            [x,rn,~,ef] = lsqlin(eye(size(x0,2)),x0(i,:)',A,b);
        else
            [x,rn,~,ef]  = lsqlin(eye(size(x0,2)),x0(i,:)',...
                A,b,Aeq,beq,lim(1,:)',lim(2,:)');
        end
    end
    xf(i,:) = x';
    resnorm(i,:) = rn;
    exitflag(i,:) = ef;
end
end