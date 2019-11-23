function [Y,I] = findNmin(X,N)
    Y = X;
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
end