A = [2,15,13,4;
    10,4,14,15;
    9,14,16,13;
    7,8,11,9];
% random matrix
A = rand(20,20);

% algorithm 1
tic
matching = Hungarian(A);
cost = sum(A(matching~=0));
toc

% algorithm 2
tic
[assign,cost] = munkres(A);
toc