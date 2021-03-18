function A = randpd(N)
% Randomly generate postive definite matrix (NxN)
D = diag(rand(N,1));
U = orth(rand(N,N));
A = U' * D * U;
end