% find N minimal number from small to big
function [Y,I] = findNmin(X,N)
%% bubble sort
Y = X(:)';
I = 1:length(Y);
M = (N<length(Y))*N+(N>=length(Y))*(length(Y));
for i=1:M
    for j=i+1:length(Y)
        if Y(i)>Y(j)
            temp = Y(i);
            Y(i) = Y(j);
            Y(j) = temp;
            temp = I(i);
            I(i) = I(j);
            I(j) = temp;
        end
    end
end
Y = Y(1:M);
I = I(1:M);
%% matlab find and min
% Z = X(:)';
% M = (N<length(Z))*N+(N>=length(Z))*(length(Z));
% I = [];
% Y = [];
% for i=1:M
%     minZ = min(Z);
%     Y = [Y,minZ];
%     I = [I,find(Z==minZ)];
%     Z(Z==minZ) = [];
% end
end